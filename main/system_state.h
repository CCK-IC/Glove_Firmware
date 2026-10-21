#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    IMU_STATE_OFF,
    IMU_STATE_INITIALIZING,
    IMU_STATE_RUNNING,
    IMU_STATE_DEGRADED,
    IMU_STATE_ERROR,
    IMU_STATE_SUSPENDED,
} imu_state_t;

typedef struct {
    uint64_t boot_time_us;
    uint32_t requested_rate_hz;
    uint32_t actual_dmp_sample_rate_hz;
    uint32_t actual_notify_rate_hz;
    uint32_t actual_raw_sample_rate_hz;
    uint32_t requested_adc_rate_hz;
    uint32_t actual_adc_sample_rate_hz;
    uint32_t status_period_ms;

    uint32_t battery_mv;
    uint8_t battery_percent;
    bool battery_low;
    bool battery_error;

    bool ble_connected;
    bool advertising;
    bool pairing;
    bool encrypted;
    bool bonded;
    bool imu_dmp;
    bool slow_mode;
    bool power_off_pending;

    uint16_t conn_handle;
    uint16_t mtu;
    uint8_t who_am_i;
    uint8_t phy;
    imu_state_t imu_state;

    uint32_t sample_count;
    uint32_t dropped_samples;
    uint32_t dropped_notifications;
    uint32_t fifo_overflows;
    uint32_t i2c_errors;
    uint32_t imu_reinitializations;
    uint32_t raw_sample_count;
    uint32_t dropped_raw_samples;
    uint32_t adc_sample_count;

    int32_t last_error;
    char last_error_module[12];
    uint8_t power_off_reason;
} system_state_t;

esp_err_t system_state_init(void);
void system_state_snapshot(system_state_t *out);
void system_state_replace(const system_state_t *in);
void system_state_set_error(const char *module, int32_t error);