#include "diagnostics.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "system_state.h"
#include "app_config.h"
#include "board/board_config.h"
#include "led/led_manager.h"

static const char *TAG = "diag";

#if CONFIG_DIAGNOSTICS_LOG_PERIOD_MS > 0
static void diagnostics_log_task(void *arg)
{
    (void)arg;
    while (true) {
        system_state_t state;
        system_state_snapshot(&state);
        ESP_LOGI(TAG,
            "board=%s fw=%d.%d.%d idf=%s ble=%d mtu=%u imu=%u dmp=%d "
            "dmp_notify=%s raw_notify=%s aux_notify=%s "
            "imu_req_hz=%lu dmp_hz=%lu raw_hz=%lu imu_notify_hz=%lu "
            "adc_req_hz=%lu adc_hz=%lu adc_notify_hz=%lu bat=%lumV/%u%% "
            "samples=%lu raw_samples=%lu adc_samples=%lu tx_drop=%lu queue_drop=%lu raw_read_fail=%lu "
            "i2c=%lu fifo=%lu heap=%lu led=%d",
            board_name(), APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH,
            esp_get_idf_version(), state.ble_connected, state.mtu, state.imu_state, state.imu_dmp,
            state.imu_dmp_subscribed ? "T" : "F",
            state.imu_raw_subscribed ? "T" : "F",
            state.aux_subscribed ? "T" : "F",
            (unsigned long)state.requested_imu_rate_hz,
            (unsigned long)state.actual_dmp_sample_rate_hz,
            (unsigned long)state.actual_raw_sample_rate_hz,
            (unsigned long)state.actual_imu_notify_rate_hz,
            (unsigned long)state.requested_adc_rate_hz,
            (unsigned long)state.actual_adc_sample_rate_hz,
            (unsigned long)state.actual_adc_notify_rate_hz,
            (unsigned long)state.battery_mv, state.battery_percent,
            (unsigned long)state.dmp_sample_count,
            (unsigned long)state.raw_sample_count,
            (unsigned long)state.adc_sample_count,
            (unsigned long)state.dropped_notifications,
            (unsigned long)state.dropped_sensor_samples,
            (unsigned long)state.dropped_raw_samples,
            (unsigned long)state.i2c_errors,
            (unsigned long)state.fifo_overflows,
            (unsigned long)esp_get_free_heap_size(),
            led_get_current_pattern());
        vTaskDelay(pdMS_TO_TICKS(CONFIG_DIAGNOSTICS_LOG_PERIOD_MS));
    }
}
#endif

esp_err_t diagnostics_start(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    ESP_LOGI(TAG, "fw=%d.%d.%d idf=%s board=%s cores=%u revision=%u BLE-only firmware",
             APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH, esp_get_idf_version(),
             board_name(), info.cores, info.revision);

#if CONFIG_DIAGNOSTICS_LOG_PERIOD_MS > 0
    return xTaskCreate(diagnostics_log_task, "diagnostics", 3072,
                       NULL, 2, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

/* Log field descriptions and sample output are maintained in SerialMSG.md. */
