#include "board_config.h"
#include "board_pins.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "board";
static i2c_master_bus_handle_t s_bus;

static bool valid_gpio(int pin)
{
    return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_GPIO(pin);
}

const char *board_name(void)
{
#if CONFIG_TARGET_BOARD_XIAO_ESP32_C3
    return "Seeed XIAO ESP32-C3";
#else
    return "Seeed XIAO ESP32-S3";
#endif
}

esp_err_t board_config_validate(void)
{
#if CONFIG_TARGET_BOARD_XIAO_ESP32_C3 && !CONFIG_IDF_TARGET_ESP32C3
#error "XIAO ESP32-C3 profile requires: idf.py set-target esp32c3"
#endif
#if CONFIG_TARGET_BOARD_XIAO_ESP32_S3 && !CONFIG_IDF_TARGET_ESP32S3
#error "XIAO ESP32-S3 profile requires: idf.py set-target esp32s3"
#endif

    const int pins[] = {
        BOARD_I2C_SCL_GPIO, BOARD_I2C_SDA_GPIO, BOARD_LED_GPIO,
        BOARD_BATTERY_ADC_GPIO, BOARD_BUTTON_GPIO, BOARD_POWER_HOLD_GPIO
    };

    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (!valid_gpio(pins[i])) {
            ESP_LOGE(TAG, "Invalid GPIO%d", pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
        for (unsigned j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
            if (pins[i] == pins[j]) {
                ESP_LOGE(TAG, "Duplicate GPIO%d assignment", pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(BOARD_BATTERY_ADC_GPIO,
                                               &unit, &channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not ADC-capable", BOARD_BATTERY_ADC_GPIO);
        return err;
    }

#if CONFIG_TARGET_BOARD_XIAO_ESP32_C3
    ESP_LOGW(TAG, "GPIO9 is a boot strapping pin; button remains input-only");
#endif
#if CONFIG_TARGET_BOARD_XIAO_ESP32_S3
    ESP_LOGW(TAG, "GPIO0 is a boot strapping pin; button remains input-only");
    if (BOARD_LED_GPIO == 43) {
        ESP_LOGW(TAG, "GPIO43 LED may conflict with UART0 console");
    }
#endif
#if CONFIG_BOARD_USE_CUSTOM_PINS
    ESP_LOGW(TAG, "Custom pins require manual flash/PSRAM/USB/JTAG review");
#endif
    return ESP_OK;
}

esp_err_t board_config_init(void)
{
    esp_err_t err = board_config_validate();
    if (err != ESP_OK) return err;

    gpio_config_t power = {
        .pin_bit_mask = 1ULL << BOARD_POWER_HOLD_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&power);
    if (err != ESP_OK) return err;
    err = gpio_set_level((gpio_num_t)BOARD_POWER_HOLD_GPIO,
                         CONFIG_POWER_HOLD_ACTIVE_LEVEL);
    if (err != ESP_OK) return err;

    gpio_config_t button = {
        .pin_bit_mask = 1ULL << BOARD_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&button);
    if (err != ESP_OK) return err;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = (gpio_num_t)BOARD_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "%s SCL=%d SDA=%d LED=%d ADC=%d BTN=%d HOLD=%d",
             board_name(), BOARD_I2C_SCL_GPIO, BOARD_I2C_SDA_GPIO,
             BOARD_LED_GPIO, BOARD_BATTERY_ADC_GPIO,
             BOARD_BUTTON_GPIO, BOARD_POWER_HOLD_GPIO);
    return ESP_OK;
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_bus;
}

bool board_button_pressed(void)
{
    const int level = gpio_get_level((gpio_num_t)BOARD_BUTTON_GPIO);
    return level == CONFIG_BOARD_BUTTON_ACTIVE_LEVEL;
}