#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t power_manager_init(void);
esp_err_t power_hold_assert(void);
esp_err_t power_hold_release(void);
void power_off_request(uint8_t reason);
void power_off_sequence(uint8_t reason);