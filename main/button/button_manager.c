#include "button_manager.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board/board_config.h"
#include "app_events.h"

static const char *TAG = "button";

static void emit(app_event_type_t type)
{
#if CONFIG_TEST_BUTTON
    ESP_LOGI(TAG, "event=%d", type);
#endif
    app_event_publish(type, 0);
}

static void task_main(void *arg)
{
    (void)arg;
    bool stable = board_button_pressed();
    bool candidate = stable;
    int64_t candidate_since = esp_timer_get_time();
    int64_t pressed_since = 0;
    int64_t last_release = 0;
    unsigned clicks = 0;
    bool hold_sent = false;

    while (true) {
        int64_t now = esp_timer_get_time();
        bool raw = board_button_pressed();

        if (raw != candidate) {
            candidate = raw;
            candidate_since = now;
        }

        if (candidate != stable &&
            now - candidate_since >= CONFIG_BUTTON_DEBOUNCE_MS * 1000LL) {
            stable = candidate;
            if (stable) {
                pressed_since = now;
                hold_sent = false;
                emit(APP_EVENT_BUTTON_PRESS);
            } else {
                emit(APP_EVENT_BUTTON_RELEASE);
                if (!hold_sent) {
                    clicks++;
                    last_release = now;
                }
            }
        }

        if (stable && !hold_sent &&
            now - pressed_since >= CONFIG_BUTTON_HOLD_MS * 1000LL) {
            hold_sent = true;
            clicks = 0;
            emit(APP_EVENT_BUTTON_HOLD);
        }

        if (!stable && clicks &&
            now - last_release >= CONFIG_BUTTON_CLICK_WINDOW_MS * 1000LL) {
            if (clicks == 1) emit(APP_EVENT_BUTTON_SINGLE);
            else if (clicks == 2) emit(APP_EVENT_BUTTON_DOUBLE);
            else emit(APP_EVENT_BUTTON_TRIPLE);
            clicks = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t button_manager_start(void)
{
    return xTaskCreate(task_main, "button", 3072, NULL, 5, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}