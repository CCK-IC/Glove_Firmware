#include "system_state.h"
#include "app_config.h"
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static system_state_t s_state;
static SemaphoreHandle_t s_lock;

esp_err_t system_state_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    memset(&s_state, 0, sizeof(s_state));
    s_state.boot_time_us = esp_timer_get_time();
    s_state.requested_rate_hz = APP_IMU_NORMAL_RATE_HZ; //modded
    s_state.status_period_ms = CONFIG_BATTERY_STATUS_PERIOD_MS;
#ifdef CONFIG_ADC_RATE_HZ
    s_state.requested_adc_rate_hz = CONFIG_ADC_RATE_HZ;
#endif
    s_state.conn_handle = UINT16_MAX;
    s_state.mtu = 23;
    s_state.phy = 0;
    return ESP_OK;
}

void system_state_snapshot(system_state_t *out)
{
    if (!out || !s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_lock);
}

void system_state_write_field(size_t offset, const void *value, size_t size)
{
    if (!s_lock || !value || offset > sizeof(s_state) ||
        size > sizeof(s_state) - offset) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy((uint8_t *)&s_state + offset, value, size);
    xSemaphoreGive(s_lock);
}

void system_state_increment_counter(size_t offset)
{
    if (!s_lock || offset > sizeof(s_state) - sizeof(uint32_t) ||
        offset % _Alignof(uint32_t)) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t *counter = (uint32_t *)((uint8_t *)&s_state + offset);
    ++*counter;
    xSemaphoreGive(s_lock);
}

void system_state_set_error(const char *module, int32_t error)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.last_error = error;
    strlcpy(s_state.last_error_module, module ? module : "unknown",
            sizeof(s_state.last_error_module));
    xSemaphoreGive(s_lock);
}
