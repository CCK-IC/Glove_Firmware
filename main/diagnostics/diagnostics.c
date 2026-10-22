#include "diagnostics.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "system_state.h"
#include "board/board_config.h"
#include "led/led_manager.h"

static const char *TAG = "diag";

static void task_main(void *arg)
{
    (void)arg;
    while (true) {
#if CONFIG_DIAGNOSTICS_LOG_PERIOD_MS > 0
        system_state_t s;
        system_state_snapshot(&s);
        ESP_LOGI(TAG,
            "board=%s idf=%s ble=%d mtu=%u imu=%u dmp=%d "
            "dmp_notify=%s raw_notify=%s aux_notify=%s "
            "rate=%lu/%lu raw=%lu notify=%lu adc=%lu bat=%lumV/%u%% "
            "samples=%lu raw_samples=%lu adc_samples=%lu drop=%lu raw_drop=%lu "
            "i2c=%lu fifo=%lu heap=%lu led=%d",
            board_name(), esp_get_idf_version(), s.ble_connected, s.mtu,
            s.imu_state, s.imu_dmp,
            s.imu_dmp_subscribed ? "T" : "F",
            s.imu_raw_subscribed ? "T" : "F",
            s.aux_subscribed ? "T" : "F",
            (unsigned long)s.requested_rate_hz,
            (unsigned long)s.actual_dmp_sample_rate_hz,
            (unsigned long)s.actual_raw_sample_rate_hz,
            (unsigned long)s.actual_notify_rate_hz,
            (unsigned long)s.actual_adc_sample_rate_hz,
            (unsigned long)s.battery_mv, s.battery_percent,
            (unsigned long)s.sample_count,
            (unsigned long)s.raw_sample_count,
            (unsigned long)s.adc_sample_count,
            (unsigned long)s.dropped_notifications,
            (unsigned long)s.dropped_raw_samples,
            (unsigned long)s.i2c_errors,
            (unsigned long)s.fifo_overflows,
            (unsigned long)esp_get_free_heap_size(),
            led_get_current_pattern());
#endif
        vTaskDelay(pdMS_TO_TICKS(CONFIG_DIAGNOSTICS_LOG_PERIOD_MS ?
                                CONFIG_DIAGNOSTICS_LOG_PERIOD_MS : 60000));
    }
}

esp_err_t diagnostics_start(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    ESP_LOGI(TAG, "IDF=%s board=%s cores=%u revision=%u BLE-only firmware",
             esp_get_idf_version(), board_name(), info.cores, info.revision);

    return xTaskCreate(task_main, "diagnostics", 3072,
                       NULL, 2, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}