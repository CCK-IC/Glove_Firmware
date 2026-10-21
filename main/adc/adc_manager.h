#pragma once
#include "esp_err.h"
#include "ble/ble_packets.h"

esp_err_t adc_manager_init(void);
esp_err_t adc_manager_start(void);
bool adc_manager_latest(adc_sample_t *sample);