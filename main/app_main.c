#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_events.h"
#include "system_state.h"
#include "board/board_config.h"
#include "imu/imu_manager.h"
#include "battery/battery_manager.h"
#include "button/button_manager.h"
#include "led/led_manager.h"
#include "power/power_manager.h"
#include "ble/ble_manager.h"
#include "diagnostics/diagnostics.h"

static const char *TAG = "app";

static void event_task(void *arg)
{
    (void)arg;
    app_event_t event;

    while (xQueueReceive(app_events_queue(), &event, portMAX_DELAY) == pdTRUE) {
        system_state_t st;

        switch (event.type) {
        case APP_EVENT_BUTTON_SINGLE:
            ESP_LOGI(TAG, "Single-click event reserved");
            break;

        case APP_EVENT_BUTTON_DOUBLE:
            system_state_snapshot(&st);
            st.slow_mode = !st.slow_mode;
            st.requested_rate_hz = st.slow_mode ?
                CONFIG_IMU_SLOW_RATE_HZ : CONFIG_IMU_RAW_RATE_HZ;
            system_state_replace(&st);
            led_set_pattern(LED_PATTERN_DEBUG_FLASH);
            ESP_LOGI(TAG, "Slow/debug mode=%d rate=%luHz",
                     st.slow_mode,
                     (unsigned long)st.requested_rate_hz);
            break;

        case APP_EVENT_BUTTON_TRIPLE:
        case APP_EVENT_POWER_OFF:
            power_off_request(event.value ? event.value : 1);
            break;

        case APP_EVENT_BUTTON_HOLD:
            ble_manager_start_pairing();
            led_set_pattern(LED_PATTERN_PAIRING);
            break;

        case APP_EVENT_PAIRING_TIMEOUT:
            led_clear_pattern(LED_PATTERN_PAIRING);
            break;

        case APP_EVENT_IMU_REINIT:
            imu_manager_fifo_reinitialize();
            break;

        default:
            break;
        }
    }
}

static void status_led_task(void *arg)
{
    (void)arg;
    bool last_connected = false;
    bool last_low = false;

    while (true) {
        system_state_t st;
        system_state_snapshot(&st);

        if (st.ble_connected != last_connected) {
            led_clear_pattern(last_connected ?
                LED_PATTERN_CONNECTED : LED_PATTERN_DISCONNECTED);
            led_set_pattern(st.ble_connected ?
                LED_PATTERN_CONNECTED : LED_PATTERN_DISCONNECTED);
            last_connected = st.ble_connected;
        }

        if (st.battery_low != last_low) {
            if (st.battery_low)
                led_set_pattern(LED_PATTERN_LOW_BATTERY);
            else
                led_clear_pattern(LED_PATTERN_LOW_BATTERY);
            last_low = st.battery_low;
        }

        if (!st.pairing) led_clear_pattern(LED_PATTERN_PAIRING);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(system_state_init());
    ESP_ERROR_CHECK(app_events_init());
    ESP_ERROR_CHECK(board_config_init());
    ESP_ERROR_CHECK(power_manager_init());

    err = led_manager_init();
    if (err != ESP_OK)
        system_state_set_error("led", err);

    led_set_pattern(LED_PATTERN_DISCONNECTED);

    ESP_ERROR_CHECK(battery_manager_init());
    ESP_ERROR_CHECK(battery_manager_start());

    err = imu_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed: %s", esp_err_to_name(err));
        system_state_set_error("imu", err);
#if !CONFIG_IMU_SYNTHETIC_FALLBACK
        return;
#endif
    }
    ESP_ERROR_CHECK(imu_manager_start());

    ESP_ERROR_CHECK(ble_manager_init());
    ESP_ERROR_CHECK(ble_manager_start());
    ESP_ERROR_CHECK(button_manager_start());
    ESP_ERROR_CHECK(diagnostics_start());

    ESP_ERROR_CHECK(xTaskCreate(event_task, "app_events", 3072,
                                NULL, 6, NULL) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(status_led_task, "status_led", 2048,
                                NULL, 3, NULL) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);

#if CONFIG_TEST_LED
    led_set_pattern(LED_PATTERN_TEST);
#endif

    ESP_LOGI(TAG, "Application started; BLE GATT only");
}