#include "adc_manager.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "board/board_pins.h"
#include "board/board_config.h"
#include "system_state.h"

static const char *TAG = "adc";
static SemaphoreHandle_t s_lock;
static adc_sample_t s_latest;
static bool s_valid;
static uint32_t s_sequence;
#if CONFIG_AUX_ADC_ENABLED && !CONFIG_ADC_DUMMY
static adc_oneshot_unit_handle_t s_units[8];
static adc_channel_t s_channels[8];
static const int s_pins[8] = {
    CONFIG_AUX_ADC_GPIO_0, CONFIG_AUX_ADC_GPIO_1,
    CONFIG_AUX_ADC_GPIO_2, CONFIG_AUX_ADC_GPIO_3,
    CONFIG_AUX_ADC_GPIO_4, CONFIG_AUX_ADC_GPIO_5,
    CONFIG_AUX_ADC_GPIO_6, CONFIG_AUX_ADC_GPIO_7,
};
static adc_atten_t attenuation(void)
{
#if CONFIG_ADC_ATTEN_0DB
    return ADC_ATTEN_DB_0;
#elif CONFIG_ADC_ATTEN_2_5DB
    return ADC_ATTEN_DB_2_5;
#elif CONFIG_ADC_ATTEN_6DB
    return ADC_ATTEN_DB_6;
#else
    return ADC_ATTEN_DB_12;
#endif
}
#endif

static void task_main(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    int64_t window = esp_timer_get_time();
    uint32_t count = 0;
    while (true) {
        system_state_t st;
        system_state_snapshot(&st);
        uint32_t rate = st.requested_adc_rate_hz ? st.requested_adc_rate_hz : 1;
        TickType_t period = pdMS_TO_TICKS(1000 / rate);
        if (!period) period = 1;
        adc_sample_t sample = {
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };
        bool ok = true;
#if CONFIG_AUX_ADC_ENABLED && !CONFIG_ADC_DUMMY
        for (unsigned channel = 0; channel < 8 && ok; ++channel) {
            int sum = 0;
            for (unsigned n = 0; n < CONFIG_ADC_AVERAGE_SAMPLES; ++n) {
                int raw;
                esp_err_t err = adc_oneshot_read(s_units[channel], s_channels[channel], &raw);
                if (err != ESP_OK) {
                    system_state_set_error("aux_adc", err);
                    ok = false;
                    break;
                }
                sum += raw;
            }
            sample.values[channel] = sum / CONFIG_ADC_AVERAGE_SAMPLES;
        }
#else
        for (unsigned channel = 0; channel < 8; ++channel)
            sample.values[channel] = (channel + 1) * 0x100;
#endif
        if (ok) {
            sample.sequence = ++s_sequence;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_latest = sample;
            s_valid = true;
            xSemaphoreGive(s_lock);
            SYSTEM_STATE_INCREMENT(adc_sample_count);
            ++count;
        }
        int64_t now = esp_timer_get_time();
        if (now - window >= 1000000) {
            SYSTEM_STATE_SET(actual_adc_sample_rate_hz,
                             count * 1000000ULL / (now - window));
            count = 0;
            window = now;
        }
        vTaskDelayUntil(&wake, period);
    }
}

esp_err_t adc_manager_init(void)
{
#if !CONFIG_AUX_ADC_ENABLED
    return ESP_OK;
#else
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
#if !CONFIG_ADC_DUMMY
    /* Validate the complete mapping before configuring any channels. */
    adc_unit_t units[8];
    const int reserved[] = {BOARD_BATTERY_ADC_GPIO, BOARD_I2C_SCL_GPIO,
        BOARD_I2C_SDA_GPIO, BOARD_LED_GPIO, BOARD_BUTTON_GPIO, BOARD_POWER_HOLD_GPIO};
    for (unsigned i = 0; i < 8; ++i) {
        for (unsigned j = 0; j < sizeof(reserved) / sizeof(reserved[0]); ++j)
            if (s_pins[i] == reserved[j]) return ESP_ERR_INVALID_ARG;
        for (unsigned j = 0; j < i; ++j)
            if (s_pins[i] == s_pins[j]) return ESP_ERR_INVALID_ARG;
        esp_err_t err = adc_oneshot_io_to_channel(s_pins[i], &units[i], &s_channels[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Configure a valid, distinct GPIO for AUX channel %u", i);
            return err;
        }
    }
    for (unsigned i = 0; i < 8; ++i) {
        esp_err_t err = board_adc_get_unit(units[i], &s_units[i]);
        if (err != ESP_OK) return err;
        adc_oneshot_chan_cfg_t cfg = {
            .bitwidth = (adc_bitwidth_t)CONFIG_ADC_WIDTH_BITS,
            .atten = attenuation(),
        };
        err = adc_oneshot_config_channel(s_units[i], s_channels[i], &cfg);
        if (err != ESP_OK) return err;
    }
#else
    ESP_LOGI(TAG, "Using dummy AUX data");
#endif
    return ESP_OK;
#endif
}

esp_err_t adc_manager_start(void)
{
#if !CONFIG_AUX_ADC_ENABLED
    return ESP_OK;
#else
    return xTaskCreate(task_main, "adc", 3072, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

bool adc_manager_latest(adc_sample_t *sample)
{
    if (!sample || !s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *sample = s_latest;
    bool valid = s_valid;
    xSemaphoreGive(s_lock);
    return valid;
}
