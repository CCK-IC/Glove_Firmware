#include "power_manager.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board_pins.h"
#include "ble/ble_manager.h"
#include "imu/imu_manager.h"
#include "led/led_manager.h"
#include "system_state.h"

static const char *TAG = "power";
static void execute_shutdown(power_off_reason_t reason);

#if CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC > 0
static TaskHandle_t s_monitor_task;

static void monitor_disconnected_timeout(void *arg)
{
    (void)arg;
    const uint64_t timeout_us = CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC * 1000000ULL;
    while (true) {
        if (system_state_try_begin_disconnected_shutdown(
                POWER_OFF_REASON_DISCONNECTED, timeout_us)) {
            ESP_LOGW(TAG, "BLE disconnected for %u seconds; powering off",
                     (unsigned)CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC);
            execute_shutdown(POWER_OFF_REASON_DISCONNECTED);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
#endif

esp_err_t power_manager_start(void)
{
#if CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC > 0
    if (s_monitor_task) return ESP_OK;
    return xTaskCreate(monitor_disconnected_timeout, "power_monitor", 3072,
                       NULL, 4, &s_monitor_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

esp_err_t power_manager_init(void)
{
    return gpio_set_level((gpio_num_t)BOARD_POWER_HOLD_GPIO,
                          CONFIG_POWER_HOLD_ACTIVE_LEVEL);
}

#if !CONFIG_POWER_LATCH_TEST_MODE
static esp_err_t release_power_latch(void)
{
    return gpio_set_level((gpio_num_t)BOARD_POWER_HOLD_GPIO,
                          !CONFIG_POWER_HOLD_ACTIVE_LEVEL);
}
#endif

void power_manager_request_shutdown(power_off_reason_t reason)
{
    if (!system_state_try_begin_shutdown(reason)) return;
    execute_shutdown(reason);
}

static void execute_shutdown(power_off_reason_t reason)
{
    ESP_LOGW(TAG, "Power-off sequence, reason=%u", reason);
    led_set_pattern(LED_PATTERN_POWER_OFF);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_POWER_OFF_LED_MS));

    ble_manager_stop_notifications();
    imu_manager_stop();
#if CONFIG_IMU_ENABLED
    esp_err_t sleep_err = imu_manager_sleep();
    if (sleep_err != ESP_OK)
        ESP_LOGW(TAG, "Could not put IMU to sleep: %s", esp_err_to_name(sleep_err));
#endif

    /* Add NVS commit/close here when persistent application settings exist. */

#if CONFIG_POWER_LATCH_TEST_MODE
    ESP_LOGW(TAG, "Latch test mode: retaining hold and entering deep sleep");
#else
    esp_err_t err = release_power_latch();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Could not release latch: %s", esp_err_to_name(err));
    else
        ESP_LOGW(TAG, "Latch released; waiting for power to drop");
#endif

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGW(TAG, "Power remains present; entering deep sleep fallback");
    esp_deep_sleep_start();

    while (true) vTaskDelay(portMAX_DELAY);
}
