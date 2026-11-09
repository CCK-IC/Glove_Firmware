#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_events.h"
#include "system_state.h"
#include "board/board_config.h"
#include "imu/imu_manager.h"
#include "adc/adc_manager.h"
#include "battery/battery_manager.h"
#include "button/button_manager.h"
#include "led/led_manager.h"
#include "power/power_manager.h"
#include "ble/ble_manager.h"
#include "diagnostics/diagnostics.h"

static const char *TAG = "app";

static void process_app_events(void *arg)
{
    (void)arg;
    app_event_t event;

    while (xQueueReceive(app_events_queue(), &event, portMAX_DELAY) == pdTRUE) {
        system_state_t state;

        switch (event.type) {
        case APP_EVENT_BUTTON_SINGLE:
            ESP_LOGI(TAG, "Single-click event reserved");
            break;

        case APP_EVENT_BUTTON_DOUBLE:
            system_state_snapshot(&state);
            state.slow_mode = !state.slow_mode;
            SYSTEM_STATE_SET(slow_mode, state.slow_mode);
            state.requested_imu_rate_hz = state.slow_mode ?
                APP_IMU_SLOW_RATE_HZ : APP_IMU_NORMAL_RATE_HZ;
            SYSTEM_STATE_SET(requested_imu_rate_hz, state.requested_imu_rate_hz);

            led_set_pattern(LED_PATTERN_DEBUG_FLASH);
            ESP_LOGI(TAG, "Slow/debug mode=%d rate=%luHz",
                     state.slow_mode,
                     (unsigned long)state.requested_imu_rate_hz);
            break;

        case APP_EVENT_BUTTON_TRIPLE:
            power_manager_request_shutdown(POWER_OFF_REASON_BUTTON);
            break;
        case APP_EVENT_POWER_OFF:
            power_manager_request_shutdown(event.value ?
                (power_off_reason_t)event.value : POWER_OFF_REASON_COMMAND);
            break;

        case APP_EVENT_BUTTON_HOLD:
            ble_manager_start_pairing();
            break;

        case APP_EVENT_PAIRING_TIMEOUT:
            led_clear_pattern(LED_PATTERN_PAIRING);
            break;

        case APP_EVENT_IMU_REINIT:
            imu_manager_reset_fifo();
            break;

        default:
            break;
        }
    }
}

static void update_status_led(void *arg)
{
    (void)arg;
    bool last_connected = false;
    bool last_battery_low = false;
    bool last_pairing = false;

    while (true) {
        system_state_t state;
        system_state_snapshot(&state);

        if (state.ble_connected != last_connected) {
            led_clear_pattern(last_connected ?
                LED_PATTERN_CONNECTED : LED_PATTERN_DISCONNECTED);
            led_set_pattern(state.ble_connected ?
                LED_PATTERN_CONNECTED : LED_PATTERN_DISCONNECTED);
            last_connected = state.ble_connected;
        }

        if (state.battery_low != last_battery_low) {
            if (state.battery_low)
                led_set_pattern(LED_PATTERN_LOW_BATTERY);
            else
                led_clear_pattern(LED_PATTERN_LOW_BATTERY);
            last_battery_low = state.battery_low;
        }

        if (state.pairing != last_pairing) {
            if (state.pairing)
                led_set_pattern(LED_PATTERN_PAIRING);
            else
                led_clear_pattern(LED_PATTERN_PAIRING);
            last_pairing = state.pairing;
        }
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
        /* Keep the disconnected shutdown active even when sensor startup fails.
         * No acquisition/host tasks are running on this path. */
        ESP_ERROR_CHECK(power_manager_start());
        return;
#endif
    }
    ESP_ERROR_CHECK(imu_manager_start());
    ESP_ERROR_CHECK(adc_manager_init());
    ESP_ERROR_CHECK(adc_manager_start());
    ESP_ERROR_CHECK(ble_manager_init());
    ESP_ERROR_CHECK(ble_manager_start());
    ESP_ERROR_CHECK(button_manager_start());
    ESP_ERROR_CHECK(diagnostics_start());

    ESP_ERROR_CHECK(xTaskCreate(process_app_events, "app_events", 3072,
                                NULL, 6, NULL) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(update_status_led, "status_led", 2048,
                                NULL, 3, NULL) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(power_manager_start());

#if CONFIG_TEST_LED
    led_set_pattern(LED_PATTERN_TEST);
#endif

    ESP_LOGI(TAG, "Application started; BLE GATT only");
}
