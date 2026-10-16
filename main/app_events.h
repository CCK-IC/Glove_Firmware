#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    APP_EVENT_BUTTON_PRESS,
    APP_EVENT_BUTTON_RELEASE,
    APP_EVENT_BUTTON_SINGLE,
    APP_EVENT_BUTTON_DOUBLE,
    APP_EVENT_BUTTON_TRIPLE,
    APP_EVENT_BUTTON_HOLD,
    APP_EVENT_PAIRING_TIMEOUT,
    APP_EVENT_POWER_OFF,
    APP_EVENT_IMU_REINIT,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    uint32_t value;
} app_event_t;

esp_err_t app_events_init(void);
QueueHandle_t app_events_queue(void);
esp_err_t app_event_publish(app_event_type_t type, uint32_t value);