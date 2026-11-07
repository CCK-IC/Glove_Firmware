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
static void power_off_execute(uint8_t reason);

#if CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC > 0
static void disconnected_monitor_task(void *arg)
{
    (void)arg;
    const uint64_t timeout_us = CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC * 1000000ULL;
    while (true) {
        if (system_state_claim_power_off(3, timeout_us)) {
            ESP_LOGW(TAG, "BLE disconnected for %u seconds; powering off",
                     (unsigned)CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC);
            power_off_execute(3);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
#endif

esp_err_t power_manager_start(void)
{
#if CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC > 0
    return xTaskCreate(disconnected_monitor_task, "power_monitor", 3072,
                       NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    return ESP_OK;
#endif
}

esp_err_t power_manager_init(void)
{
    return power_hold_assert();
}

esp_err_t power_hold_assert(void)
{
    return gpio_set_level((gpio_num_t)BOARD_POWER_HOLD_GPIO,
                          CONFIG_POWER_HOLD_ACTIVE_LEVEL);
}

esp_err_t power_hold_release(void)
{
    return gpio_set_level((gpio_num_t)BOARD_POWER_HOLD_GPIO,
                          !CONFIG_POWER_HOLD_ACTIVE_LEVEL);
}

void power_off_request(uint8_t reason)
{
    power_off_sequence(reason);
}

void power_off_sequence(uint8_t reason)
{
    if (!system_state_claim_power_off(reason, 0)) return;
    power_off_execute(reason);
}

static void power_off_execute(uint8_t reason)
{
    ESP_LOGW(TAG, "Power-off sequence, reason=%u", reason);
    led_set_pattern(LED_PATTERN_POWER_OFF);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_POWER_OFF_LED_MS));

    ble_manager_stop_notifications();
    imu_manager_stop();
#if CONFIG_IMU_ENABLED
    imu_manager_sleep();
#endif

    /* Add NVS commit/close here when persistent application settings exist. */

#if CONFIG_POWER_LATCH_TEST_MODE
    ESP_LOGW(TAG, "Latch test mode: retaining hold and entering deep sleep");
#else
    esp_err_t err = power_hold_release();
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
