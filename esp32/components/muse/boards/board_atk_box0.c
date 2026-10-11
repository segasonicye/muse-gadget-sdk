/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Alientek ATK-DNESP32S3-BOX0: ESP32-S3 (QFN56 rev v0.2) with 16 MB quad
 * flash and 8 MB embedded octal PSRAM, native USB Serial/JTAG. A 240x240
 * ST7789 on SPI, an ES8311 codec (no MCLK: the codec recovers its clock
 * from BCLK; no external PA), three buttons and a LiPo behind a SYS_POW
 * latch, read through an ADC divider gated by CHG_CTRL. Pins follow the
 * vendor's xiaozhi-esp32 board support (boards/alientek/atk-dnesp32s3-box0:
 * config.h and atk_dnesp32s3_box0.cc), cross-checked on the bench.
 */
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/ledc.h"
#include "driver/rtc_io.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_mem.h"
#include "muse_state.h"

static const char *TAG = "board";

#define LCD_W 240
#define LCD_H 240
#define LCD_HOST SPI3_HOST
#define LCD_MOSI GPIO_NUM_40
#define LCD_SCLK GPIO_NUM_39
#define LCD_DC GPIO_NUM_38
#define LCD_CS GPIO_NUM_41
#define LCD_RST GPIO_NUM_NC          /* the panel's reset line is not wired */
#define LCD_BL GPIO_NUM_42
#define BL_HZ 20000
#define DRAW_BUF_LINES 32

#define TALK_GPIO GPIO_NUM_4         /* middle button: talk/PTT (xiaozhi main button: toggles chat) */
#define MENU_GPIO GPIO_NUM_3         /* left button: menu */
#define PWR_GPIO GPIO_NUM_0          /* right button: short press menu up, hold for aux/menu/power (also the boot strap) */
                                     /* (active high, per the vendor board support) */

#define SYS_POW GPIO_NUM_2           /* power latch: high keeps the board on */
#define CODEC_PWR GPIO_NUM_14        /* ES8311 supply rail */
#define CHRG_GPIO GPIO_NUM_48        /* low while charging */
#define CHG_CTRL GPIO_NUM_47         /* gates the battery divider; high to save power */
#define BATT_ADC_UNIT ADC_UNIT_1
#define BATT_ADC_CHAN ADC_CHANNEL_0  /* GPIO1 */

#define I2C_SDA GPIO_NUM_11
#define I2C_SCL GPIO_NUM_12
#define I2S_BCLK GPIO_NUM_5
#define I2S_WS GPIO_NUM_10
#define I2S_DOUT GPIO_NUM_9
#define I2S_DIN GPIO_NUM_6

static i2c_master_bus_handle_t s_i2c;
static esp_lcd_panel_handle_t s_panel;
static muse_gpio_button_t s_talk, s_menu, s_pwr;
static adc_oneshot_unit_handle_t s_adc;
static muse_power_t s_power_cache = { .battery_pct = -1 };

static esp_err_t init(void)
{
    /* The power button turns the board on through the SYS_POW latch; hold it,
     * as the vendor firmware does, so battery power stays on (and survives
     * light sleep, which powers down the RTC pad registers). */
    rtc_gpio_init(SYS_POW);
    rtc_gpio_set_direction(SYS_POW, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(SYS_POW, 1);
    rtc_gpio_hold_en(SYS_POW);

    /* The codec's supply rail and the battery divider gate come up on. */
    const gpio_config_t rails = {
        .pin_bit_mask = BIT64(CODEC_PWR) | BIT64(CHG_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rails), TAG, "power rails");
    gpio_set_level(CODEC_PWR, 1);
    gpio_set_level(CHG_CTRL, 1);

    const gpio_config_t chg = {
        .pin_bit_mask = BIT64(CHRG_GPIO),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&chg), TAG, "charge pin");

    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");

    /* The middle (talk) button reads high when pressed, unlike the other two. */
    ESP_RETURN_ON_ERROR(muse_gpio_button_init_high(&s_talk, TALK_GPIO), TAG, "talk button");
    /* muse_gpio_button_init_high() sets no pull; xiaozhi iot_button enables
     * the internal pull-down for active-high buttons. Without it GPIO4 floats
     * and the middle button never produces an edge. */
    gpio_pulldown_en(TALK_GPIO);
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_menu, MENU_GPIO), TAG, "menu button");
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_pwr, PWR_GPIO), TAG, "aux button");
    /* GPIO0 (aux, right) is the boot strap: a press during reset enters the
     * ROM loader. At runtime it is just an input with a pull-up; nothing
     * drives it low. */
    s_talk.pressed = gpio_get_level(TALK_GPIO) == 1;
    s_menu.pressed = gpio_get_level(MENU_GPIO) == 0;
    s_pwr.pressed = gpio_get_level(PWR_GPIO) == 0;

    const adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = BATT_ADC_UNIT };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc");
    const adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    return adc_oneshot_config_channel(s_adc, BATT_ADC_CHAN, &ch_cfg);
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    (void)touch;
    const ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = BL_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    if (ledc_timer_config(&bl_timer) != ESP_OK || ledc_channel_config(&bl_ch) != ESP_OK) {
        return NULL;
    }

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_W * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 80 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
    };
    if (esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_W,
            .ver_res = LCD_H,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pct * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

/* One ES8311 does both directions over a duplex I2S bus. There is no MCLK
 * (use_mclk = false: the codec recovers its clock from BCLK, as the vendor
 * board support runs it) and no external PA (pa_pin unused). Muse records
 * two slots; the mic answers on the left one. */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");
    const i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .rx_handle = rx, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_codec = { .port = I2C_NUM_0, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_codec);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(data_if && ctrl_if && gpio_if, ESP_ERR_NO_MEM, TAG, "codec interfaces");

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = GPIO_NUM_NC,      /* no external amplifier on this board */
        .use_mclk = false,
        .hw_gain = { .pa_voltage = 3.3, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *codec = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec, ESP_FAIL, TAG, "ES8311 not responding");

    esp_codec_dev_cfg_t out_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = codec, .data_if = data_if };
    esp_codec_dev_cfg_t in_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN, .codec_if = codec, .data_if = data_if };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}

static unsigned poll_buttons(void)
{
    unsigned ev = muse_gpio_button_poll(&s_talk);
    /* Right button: a short press is menu UP (fires on release); holding it
     * for ~1.2 s turns it into the aux key (menu/power), so the aux actions
     * stay reachable. Left button is menu DOWN. */
    static TickType_t s_up_since;
    static bool s_up_aux;
    unsigned r = muse_gpio_button_poll(&s_pwr);
    if (r & MUSE_BTN_TALK_PRESS) {
        s_up_since = xTaskGetTickCount();
        s_up_aux = false;
    } else if (r & MUSE_BTN_TALK_RELEASE) {
        if (s_up_aux) {
            ev |= MUSE_BTN_AUX_RELEASE;
            s_up_aux = false;
        } else {
            ev |= MUSE_BTN_UP;
        }
    } else if (s_pwr.pressed && !s_up_aux &&
               xTaskGetTickCount() - s_up_since >= pdMS_TO_TICKS(1200)) {
        ev |= MUSE_BTN_AUX_PRESS;
        s_up_aux = true;
    }
    if (muse_gpio_button_poll(&s_menu) & MUSE_BTN_TALK_PRESS) {
        ev |= MUSE_BTN_DOWN;
    }
    return ev;
}

/* The vendor raw-ADC level table for this board (xiaozhi's power_manager.h),
 * linearly interpolated. The divider sits behind CHG_CTRL: gate it on, let
 * it settle, average ten reads, gate it off again. CHRG reads low while
 * charging. */
static esp_err_t read_power(muse_power_t *out)
{
    static const struct { uint16_t adc; uint8_t level; } levels[] = {
        { 2951, 0 }, { 3019, 20 }, { 3037, 40 }, { 3091, 60 }, { 3124, 80 }, { 3231, 100 },
    };
    bool charging = gpio_get_level(CHRG_GPIO) == 0;
    gpio_set_level(CHG_CTRL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    int sum = 0, n = 0;
    for (int i = 0; i < 10; i++) {
        int raw;
        if (adc_oneshot_read(s_adc, BATT_ADC_CHAN, &raw) == ESP_OK) {
            sum += raw;
            n++;
        }
    }
    gpio_set_level(CHG_CTRL, 1);
    if (n == 0) {
        *out = s_power_cache;
        return ESP_OK;
    }
    int raw = sum / n;
    muse_power_t p = { .battery_pct = 0, .battery_mv = 0, .charging = false, .usb = false };
    if (raw < levels[0].adc) {
        p.battery_pct = 0;
    } else if (raw >= levels[5].adc) {
        p.battery_pct = 100;
    } else {
        for (int i = 0; i < 5; i++) {
            if (raw >= levels[i].adc && raw < levels[i + 1].adc) {
                p.battery_pct = levels[i].level +
                    (raw - levels[i].adc) * (levels[i + 1].level - levels[i].level) / (levels[i + 1].adc - levels[i].adc);
                break;
            }
        }
    }
    p.charging = charging && p.battery_pct < 100;
    p.usb = charging;
    s_power_cache = p;
    *out = p;
    return ESP_OK;
}

static esp_err_t power_off(void)
{
    set_brightness(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    gpio_set_level(CHG_CTRL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    /* Opening the latch cuts battery power once the button is up; deep sleep
     * covers USB power. The middle (talk) button turns the board back on. */
    rtc_gpio_set_level(SYS_POW, 0);
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext0_wakeup(TALK_GPIO, 1), TAG, "talk button wake");
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "ATK BOX0",
    .width = LCD_W,
    .height = LCD_H,
    .round = false,
    .touch = false,
    .diagonal_in = 1.54f,
    .keyboard = false,
    .talk_button = "middle",
    .aux_button = "right",
    .talk_hint = { LV_ALIGN_BOTTOM_MID, 0, -4 },
    .aux_hint = { LV_ALIGN_BOTTOM_RIGHT, -14, -4 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .audio_init = audio_init,
    .mic_slot = 0,              /* one mic, on the left slot */
    .poll_buttons = poll_buttons,
    .read_power = read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
