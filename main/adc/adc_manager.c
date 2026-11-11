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
static SemaphoreHandle_t s_sample_lock;
static adc_sample_t s_latest_sample;
static bool s_has_sample;
static uint32_t s_sample_sequence;
#if CONFIG_AUX_ADC_ENABLED && !CONFIG_ADC_DUMMY
static adc_oneshot_unit_handle_t s_adc_units[APP_AUX_ADC_CHANNEL_COUNT];
static adc_channel_t s_adc_channels[APP_AUX_ADC_CHANNEL_COUNT];
static const int s_channel_gpios[APP_AUX_ADC_CHANNEL_COUNT] = {
    CONFIG_AUX_ADC_GPIO_0,
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 1
    CONFIG_AUX_ADC_GPIO_1,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 2
    CONFIG_AUX_ADC_GPIO_2,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 3
    CONFIG_AUX_ADC_GPIO_3,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 4
    CONFIG_AUX_ADC_GPIO_4,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 5
    CONFIG_AUX_ADC_GPIO_5,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 6
    CONFIG_AUX_ADC_GPIO_6,
#endif
#if CONFIG_AUX_ADC_CHANNEL_COUNT > 7
    CONFIG_AUX_ADC_GPIO_7,
#endif
};
static adc_atten_t configured_attenuation(void)
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

static void adc_sampling_task(void *arg)
{
    (void)arg;
    TickType_t last_wake_tick = xTaskGetTickCount();
    int64_t rate_window_start_us = esp_timer_get_time();
    uint32_t samples_in_window = 0;
    while (true) {
        system_state_t state;
        system_state_snapshot(&state);
        uint32_t sample_rate_hz = state.requested_adc_rate_hz ?
            state.requested_adc_rate_hz : 1;
        TickType_t sample_period_ticks = pdMS_TO_TICKS(1000 / sample_rate_hz);
        if (!sample_period_ticks) sample_period_ticks = 1;
        /* Aggregate initialization keeps every unused packet channel at zero. */
        adc_sample_t sample = {
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };
        bool sample_valid = true;
#if CONFIG_AUX_ADC_ENABLED && !CONFIG_ADC_DUMMY
        for (unsigned channel = 0; channel < APP_AUX_ADC_CHANNEL_COUNT && sample_valid; ++channel) {
            int raw_sum = 0;
            for (unsigned n = 0; n < CONFIG_ADC_AVERAGE_SAMPLES; ++n) {
                int raw_count;
                esp_err_t err = adc_oneshot_read(s_adc_units[channel],
                                                s_adc_channels[channel], &raw_count);
                if (err != ESP_OK) {
                    system_state_set_error("aux_adc", err);
                    sample_valid = false;
                    break;
                }
                raw_sum += raw_count;
            }
            sample.values[channel] = raw_sum / CONFIG_ADC_AVERAGE_SAMPLES;
        }
#elif CONFIG_AUX_ADC_ENABLED
        for (unsigned channel = 0; channel < APP_AUX_ADC_CHANNEL_COUNT; ++channel)
            sample.values[channel] = (channel + 1) * 0x100;
#endif
        if (sample_valid) {
            sample.sequence = ++s_sample_sequence;
            xSemaphoreTake(s_sample_lock, portMAX_DELAY);
            s_latest_sample = sample;
            s_has_sample = true;
            xSemaphoreGive(s_sample_lock);
            SYSTEM_STATE_INCREMENT(adc_sample_count);
            ++samples_in_window;
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us - rate_window_start_us >= 1000000) {
            SYSTEM_STATE_SET(actual_adc_sample_rate_hz,
                             samples_in_window * 1000000ULL /
                                 (now_us - rate_window_start_us));
            samples_in_window = 0;
            rate_window_start_us = now_us;
        }
        vTaskDelayUntil(&last_wake_tick, sample_period_ticks);
    }
}

esp_err_t adc_manager_init(void)
{
#if !CONFIG_AUX_ADC_ENABLED
    return ESP_OK;
#else
    s_sample_lock = xSemaphoreCreateMutex();
    if (!s_sample_lock) return ESP_ERR_NO_MEM;
#if !CONFIG_ADC_DUMMY
    /* Validate the complete mapping before configuring any channels. */
    adc_unit_t channel_units[APP_AUX_ADC_CHANNEL_COUNT];
    const int reserved[] = {BOARD_BATTERY_ADC_GPIO, BOARD_I2C_SCL_GPIO,
        BOARD_I2C_SDA_GPIO, BOARD_LED_GPIO, BOARD_BUTTON_GPIO, BOARD_POWER_HOLD_GPIO};
    for (unsigned i = 0; i < APP_AUX_ADC_CHANNEL_COUNT; ++i) {
        for (unsigned j = 0; j < sizeof(reserved) / sizeof(reserved[0]); ++j)
            if (s_channel_gpios[i] == reserved[j]) {
                ESP_LOGE(TAG, "AUX channel %u GPIO%d is reserved by the board", i,
                         s_channel_gpios[i]);
                return ESP_ERR_INVALID_ARG;
            }
        for (unsigned j = 0; j < i; ++j)
            if (s_channel_gpios[i] == s_channel_gpios[j]) return ESP_ERR_INVALID_ARG;
        esp_err_t err = adc_oneshot_io_to_channel(s_channel_gpios[i],
                                                &channel_units[i], &s_adc_channels[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Configure a valid, distinct GPIO for AUX channel %u", i);
            return err;
        }
    }
    for (unsigned i = 0; i < APP_AUX_ADC_CHANNEL_COUNT; ++i) {
        esp_err_t err = board_adc_get_unit(channel_units[i], &s_adc_units[i]);
        if (err != ESP_OK) return err;
        adc_oneshot_chan_cfg_t cfg = {
            .bitwidth = (adc_bitwidth_t)CONFIG_ADC_WIDTH_BITS,
            .atten = configured_attenuation(),
        };
        /* AUX protocol carries raw counts, so voltage calibration is unnecessary. */
        err = adc_oneshot_config_channel(s_adc_units[i], s_adc_channels[i], &cfg);
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
    if (!s_sample_lock) return ESP_ERR_INVALID_STATE;
    return xTaskCreate(adc_sampling_task, "adc", 3072, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

bool adc_manager_latest(adc_sample_t *sample)
{
    if (!sample || !s_sample_lock) return false;
    xSemaphoreTake(s_sample_lock, portMAX_DELAY);
    *sample = s_latest_sample;
    bool valid = s_has_sample;
    xSemaphoreGive(s_sample_lock);
    return valid;
}
