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
    s_state.ble_disconnected_since_us = s_state.boot_time_us;
    s_state.requested_imu_rate_hz = APP_IMU_NORMAL_RATE_HZ;
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

void system_state_set_ble_connected(bool connected)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state.ble_connected != connected) {
        s_state.ble_connected = connected;
        if (!connected) s_state.ble_disconnected_since_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_lock);
}

void system_state_update_battery(uint32_t millivolts, uint8_t percent, bool low)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.battery_mv = millivolts;
    s_state.battery_percent = percent;
    s_state.battery_low = low;
    s_state.battery_error = false;
    xSemaphoreGive(s_lock);
}

static bool try_begin_shutdown(uint8_t reason, uint64_t disconnected_timeout_us)
{
    if (!s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool due = disconnected_timeout_us == 0 ||
        (!s_state.ble_connected &&
         (uint64_t)esp_timer_get_time() - s_state.ble_disconnected_since_us >=
             disconnected_timeout_us);
    bool claimed = due && !s_state.power_off_pending;
    if (claimed) {
        s_state.power_off_pending = true;
        s_state.power_off_reason = reason;
    }
    xSemaphoreGive(s_lock);
    return claimed;
}

bool system_state_try_begin_shutdown(uint8_t reason)
{
    return try_begin_shutdown(reason, 0);
}

bool system_state_try_begin_disconnected_shutdown(uint8_t reason, uint64_t timeout_us)
{
    /* Zero disables the disconnected timer; it must not claim a manual shutdown. */
    return timeout_us > 0 && try_begin_shutdown(reason, timeout_us);
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
