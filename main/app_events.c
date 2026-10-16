#include "app_events.h"
#include "app_config.h"

static QueueHandle_t s_queue;

esp_err_t app_events_init(void)
{
    s_queue = xQueueCreate(APP_EVENT_QUEUE_LEN, sizeof(app_event_t));
    return s_queue ? ESP_OK : ESP_ERR_NO_MEM;
}

QueueHandle_t app_events_queue(void)
{
    return s_queue;
}

esp_err_t app_event_publish(app_event_type_t type, uint32_t value)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    app_event_t e = {.type = type, .value = value};
    return xQueueSend(s_queue, &e, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}