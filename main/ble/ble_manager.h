#pragma once

#include "esp_err.h"

esp_err_t ble_manager_init(void);
esp_err_t ble_manager_start(void);
void ble_manager_start_pairing(void);
void ble_manager_stop_notifications(void);