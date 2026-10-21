#include "adc_manager.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board_pins.h"
#include "system_state.h"

static const char *TAG = "adc";
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;
static bool s_calibrated;
static SemaphoreHandle_t s_adc_sample_lock;
static adc_sample_t s_latest_adc_sample;
static uint32_t s_sequence;

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

static void publish_adc(const adc_sample_t *sample)
{
    xSemaphoreTake(s_adc_sample_lock, portMAX_DELAY);
    s_latest_adc_sample = *sample;
    xSemaphoreGive(s_adc_sample_lock);

    system_state_t st;
    system_state_snapshot(&st);
    st.adc_sample_count++;
    system_state_replace(&st);
}

static void task_main(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint64_t count_window = esp_timer_get_time();
    uint32_t count = 0;
    while (true) {
        system_state_t st;
        system_state_snapshot(&st);
        uint32_t rate = st.requested_adc_rate_hz ? st.requested_adc_rate_hz : 1;
        TickType_t period = pdMS_TO_TICKS(1000 / rate);
        if (!period) period = 1;

        // int sum = 0;
        // bool ok = true;
#ifndef CONFIG_ADC_DUMMY  
        for (int i = 0; i < CONFIG_BATTERY_AVERAGE_SAMPLES; ++i) {
            int raw;
            if (adc_oneshot_read(s_adc, s_channel, &raw) != ESP_OK) {
                ok = false;
                break;
            }
            sum += raw;
        }




    #if CONFIG_TEST_ADC_PRINT
            ESP_LOGI(TAG, "raw=%d adc=%dmV battery=%lumV percent=%u",
                     raw, adc_mv, (unsigned long)st.battery_mv,
                     st.battery_percent);
    #endif
#else
        /* DUMMY code */
        adc_sample_t dummy = {
            .sequence = ++s_sequence,
            .timestamp_ms = esp_timer_get_time() / 1000ULL,
            .values = {0x100, 0x200, 0x300, 0x400, 0x500, 0x600, 0x700, 0x800},
            // .adc1 = 1000,
            // .adc2 = 2000,
            // .adc3 = 3000,
            // .adc4 = 4000,
            // .adc5 = 5000,
            // .adc6 = 6000,
            // .adc7 = 7000,
            // .adc8 = 8000,
        };
        publish_adc(&dummy);
        count++;
#endif
        system_state_replace(&st);
        const uint64_t now = esp_timer_get_time();
        if (now - count_window >= 1000000ULL) {
            system_state_snapshot(&st);
    #ifndef  CONFIG_ADC_DUMMY
            st.actual_adc_sample_rate_hz = count;        
    #endif
            system_state_replace(&st);
#ifndef CONFIG_ADC_DUMMY

            count = 0;
#endif
            
            count = 0;
            count_window = now;
        }
        vTaskDelayUntil(&wake, period);
    }
}

esp_err_t adc_manager_init(void)
{
    s_adc_sample_lock = xSemaphoreCreateMutex();
    if (!s_adc_sample_lock) {
        return ESP_ERR_NO_MEM;
    }
#ifndef CONFIG_ADC_DUMMY    
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(BOARD_BATTERY_ADC_GPIO,
                                               &unit, &s_channel);
    if (err != ESP_OK) return err;

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = unit};
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = attenuation(),
    };
    err = adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg);
    if (err != ESP_OK) return err;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = unit,
        .chan = s_channel,
        .atten = attenuation(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_calibrated =
        adc_cali_create_scheme_curve_fitting(&cal, &s_cali) == ESP_OK;
    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cal = {
        .unit_id = unit,
        .atten = attenuation(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 0,
    };
    s_calibrated =
        adc_cali_create_scheme_line_fitting(&cal, &s_cali) == ESP_OK;
    #endif
#endif
    if (!s_calibrated)
        ESP_LOGW(TAG, "ADC calibration unavailable; voltage is approximate");
    return ESP_OK;
}

esp_err_t adc_manager_start(void)
{
    return xTaskCreate(task_main, "adc", 3072, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

bool adc_manager_latest(adc_sample_t *sample)
{
    if (!sample || !s_adc_sample_lock) return false;

    xSemaphoreTake(s_adc_sample_lock, portMAX_DELAY);
    *sample = s_latest_adc_sample;
    bool valid = 1;
        // (s_latest_adc_sample.flags & IMU_RAW_FLAG_VALID) != 0;
    xSemaphoreGive(s_adc_sample_lock);

    return valid;
}