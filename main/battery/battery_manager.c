#include "battery_manager.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board_pins.h"
#include "board/board_config.h"
#include "system_state.h"

static const char *TAG = "battery";
static adc_oneshot_unit_handle_t s_adc_unit;
static adc_cali_handle_t s_calibration;
static adc_channel_t s_adc_channel;
static bool s_has_calibration;

static adc_atten_t configured_attenuation(void)
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

static uint8_t voltage_to_percent(uint32_t battery_mv)
{
    if (battery_mv <= CONFIG_BATTERY_EMPTY_MV) return 0;
    if (battery_mv >= CONFIG_BATTERY_FULL_MV) return 100;
    return (uint8_t)(((battery_mv - CONFIG_BATTERY_EMPTY_MV) * 100U) /
        (CONFIG_BATTERY_FULL_MV - CONFIG_BATTERY_EMPTY_MV));
}

static esp_err_t read_averaged_adc(int *raw_count, int *adc_mv)
{
    int raw_sum = 0;
    for (unsigned i = 0; i < CONFIG_BATTERY_AVERAGE_SAMPLES; ++i) {
        int reading;
        esp_err_t err = adc_oneshot_read(s_adc_unit, s_adc_channel, &reading);
        if (err != ESP_OK) return err;
        raw_sum += reading;
    }

    *raw_count = raw_sum / CONFIG_BATTERY_AVERAGE_SAMPLES;
    if (s_has_calibration)
        return adc_cali_raw_to_voltage(s_calibration, *raw_count, adc_mv);

    /* Uncalibrated approximation for diagnostics only. */
    *adc_mv = (*raw_count * 3300) / 4095;
    return ESP_OK;
}

static uint32_t adc_to_battery_mv(int adc_mv)
{
    int64_t battery_mv =
        (int64_t)adc_mv * CONFIG_BATTERY_DIVIDER_NUMERATOR /
        CONFIG_BATTERY_DIVIDER_DENOMINATOR;
    battery_mv = battery_mv * CONFIG_BATTERY_CALIBRATION_GAIN_PERMILLE / 1000;
    battery_mv += CONFIG_BATTERY_CALIBRATION_OFFSET_MV;
    return battery_mv > 0 ? (uint32_t)battery_mv : 0;
}

static void battery_sampling_task(void *arg)
{
    (void)arg;
    while (true) {
        int raw_count = 0;
        int adc_mv = 0;
        esp_err_t err = read_averaged_adc(&raw_count, &adc_mv);
        if (err == ESP_OK) {
            uint32_t battery_mv = adc_to_battery_mv(adc_mv);
            uint8_t battery_percent = voltage_to_percent(battery_mv);
            system_state_update_battery(battery_mv, battery_percent,
                                        battery_mv <= CONFIG_BATTERY_LOW_MV);

#if CONFIG_TEST_BATTERY
            ESP_LOGI(TAG, "raw=%d adc=%dmV battery=%lumV percent=%u",
                     raw_count, adc_mv, (unsigned long)battery_mv,
                     battery_percent);
#endif
        } else {
            /* Preserve the last valid reading on ADC or calibration failure. */
            SYSTEM_STATE_SET(battery_error, true);
            system_state_set_error("battery", err);
        }

        system_state_t state;
        system_state_snapshot(&state);
        TickType_t period_ticks = pdMS_TO_TICKS(state.status_period_ms);
        vTaskDelay(period_ticks ? period_ticks : 1);
    }
}

esp_err_t battery_manager_init(void)
{
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(BOARD_BATTERY_ADC_GPIO,
                                             &unit, &s_adc_channel);
    if (err != ESP_OK) return err;

    err = board_adc_get_unit(unit, &s_adc_unit);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = configured_attenuation(),
    };
    err = adc_oneshot_config_channel(s_adc_unit, s_adc_channel, &chan_cfg);
    if (err != ESP_OK) return err;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = unit,
        .chan = s_adc_channel,
        .atten = configured_attenuation(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_has_calibration =
        adc_cali_create_scheme_curve_fitting(&cal, &s_calibration) == ESP_OK;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cal = {
        .unit_id = unit,
        .atten = configured_attenuation(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 0,
    };
    s_has_calibration =
        adc_cali_create_scheme_line_fitting(&cal, &s_calibration) == ESP_OK;
#endif

    if (!s_has_calibration)
        ESP_LOGW(TAG, "ADC calibration unavailable; voltage is approximate");
    return ESP_OK;
}

esp_err_t battery_manager_start(void)
{
    if (!s_adc_unit) return ESP_ERR_INVALID_STATE;
    return xTaskCreate(battery_sampling_task, "battery", 3072, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}
