#include "battery_manager.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board_pins.h"
#include "system_state.h"

static const char *TAG = "battery";
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;
static bool s_calibrated;

static adc_atten_t attenuation(void)
{
#if CONFIG_BATTERY_ADC_ATTEN_0DB
    return ADC_ATTEN_DB_0;
#elif CONFIG_BATTERY_ADC_ATTEN_2_5DB
    return ADC_ATTEN_DB_2_5;
#elif CONFIG_BATTERY_ADC_ATTEN_6DB
    return ADC_ATTEN_DB_6;
#else
    return ADC_ATTEN_DB_12;
#endif
}

static uint8_t percentage(uint32_t mv)
{
    if (mv <= CONFIG_BATTERY_EMPTY_MV) return 0;
    if (mv >= CONFIG_BATTERY_FULL_MV) return 100;
    return (uint8_t)(((mv - CONFIG_BATTERY_EMPTY_MV) * 100U) /
        (CONFIG_BATTERY_FULL_MV - CONFIG_BATTERY_EMPTY_MV));
}

static void task_main(void *arg)
{
    (void)arg;
    while (true) {
        int sum = 0;
        bool ok = true;
        for (int i = 0; i < CONFIG_BATTERY_AVERAGE_SAMPLES; ++i) {
            int raw;
            if (adc_oneshot_read(s_adc, s_channel, &raw) != ESP_OK) {
                ok = false;
                break;
            }
            sum += raw;
        }

        system_state_t st;
        system_state_snapshot(&st);

        if (ok) {
            int raw = sum / CONFIG_BATTERY_AVERAGE_SAMPLES;
            int adc_mv = 0;
            if (s_calibrated) {
                ok = adc_cali_raw_to_voltage(s_cali, raw, &adc_mv) == ESP_OK;
            } else {
                /* Uncalibrated approximation for diagnostics only. */
                adc_mv = (raw * 3300) / 4095;
            }

            int64_t battery_mv =
                (int64_t)adc_mv * CONFIG_BATTERY_DIVIDER_NUMERATOR /
                CONFIG_BATTERY_DIVIDER_DENOMINATOR;
            battery_mv = battery_mv *
                CONFIG_BATTERY_CALIBRATION_GAIN_PERMILLE / 1000;
            battery_mv += CONFIG_BATTERY_CALIBRATION_OFFSET_MV;
            if (battery_mv < 0) battery_mv = 0;

            st.battery_mv = battery_mv;
            st.battery_percent = percentage(st.battery_mv);
            st.battery_low = st.battery_mv <= CONFIG_BATTERY_LOW_MV;
            st.battery_error = false;

#if CONFIG_TEST_BATTERY
            ESP_LOGI(TAG, "raw=%d adc=%dmV battery=%lumV percent=%u",
                     raw, adc_mv, (unsigned long)st.battery_mv,
                     st.battery_percent);
#endif
        } else {
            st.battery_error = true;
        }

        system_state_replace(&st);
        vTaskDelay(pdMS_TO_TICKS(st.status_period_ms));
    }
}

esp_err_t battery_manager_init(void)
{
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

    if (!s_calibrated)
        ESP_LOGW(TAG, "ADC calibration unavailable; voltage is approximate");
    return ESP_OK;
}

esp_err_t battery_manager_start(void)
{
    return xTaskCreate(task_main, "battery", 3072, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}