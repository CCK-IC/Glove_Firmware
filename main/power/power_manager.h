#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    POWER_OFF_REASON_BUTTON = 1,
    POWER_OFF_REASON_COMMAND = 2,
    POWER_OFF_REASON_DISCONNECTED = 3,
} power_off_reason_t;

esp_err_t power_manager_init(void);
esp_err_t power_manager_start(void);
/* Runs the shutdown sequence in the calling task; do not call from callbacks. */
void power_manager_request_shutdown(power_off_reason_t reason);
