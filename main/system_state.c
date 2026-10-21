#include "system_state.h"
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
    s_state.requested_rate_hz = CONFIG_IMU_RAW_RATE_HZ; //modded
    s_state.status_period_ms = CONFIG_BATTERY_STATUS_PERIOD_MS;
    s_state.requested_adc_rate_hz = CONFIG_ADC_RATE_HZ;
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

void system_state_replace(const system_state_t *in)
{
    if (!in || !s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = *in;
    xSemaphoreGive(s_lock);
}

void system_state_set_error(const char *module, int32_t error)
{
    system_state_t st;
    system_state_snapshot(&st);
    st.last_error = error;
    strlcpy(st.last_error_module, module ? module : "unknown",
            sizeof(st.last_error_module));
    system_state_replace(&st);
}