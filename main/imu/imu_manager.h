#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "ble/ble_packets.h"

esp_err_t imu_manager_init(void);
esp_err_t imu_manager_start(void);
void imu_manager_stop(void);
esp_err_t imu_manager_sleep(void);
esp_err_t imu_manager_fifo_reinitialize(void);
bool imu_manager_latest(imu_dmp_sample_t *sample);
bool imu_manager_latest_raw(imu_raw_sample_t *sample);