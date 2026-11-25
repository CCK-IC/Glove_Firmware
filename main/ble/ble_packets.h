#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "app_config.h"

enum {
    IMU_FLAG_VALID      = 1U << 0,
    IMU_FLAG_DMP        = 1U << 1,
    IMU_FLAG_SYNTHETIC  = 1U << 2,
    IMU_FLAG_FIFO_RESET = 1U << 3,
    IMU_FLAG_DEGRADED   = 1U << 4,
};

enum {
    IMU_RAW_FLAG_VALID          = 1U << 0,
    IMU_RAW_FLAG_ACCEL_VALID    = 1U << 1,
    IMU_RAW_FLAG_GYRO_VALID     = 1U << 2,
    IMU_RAW_FLAG_MAG_VALID      = 1U << 3,
    IMU_RAW_FLAG_TEMP_VALID     = 1U << 4,
    IMU_RAW_FLAG_MAG_OVERFLOW   = 1U << 5,
    IMU_RAW_FLAG_DMP_CONCURRENT = 1U << 6,
};

#define AUX_PACKET_CHANNEL_COUNT 8U

_Static_assert(APP_AUX_ADC_CHANNEL_COUNT <= AUX_PACKET_CHANNEL_COUNT,
               "Too many AUX ADC channels");
#if CONFIG_AUX_ADC_ENABLED
_Static_assert(APP_AUX_ADC_CHANNEL_COUNT >= 1,
               "At least one AUX ADC channel is required");
#if CONFIG_IDF_TARGET_ESP32C3
_Static_assert(APP_AUX_ADC_CHANNEL_COUNT <= 3,
               "ESP32-C3 supports at most three AUX channels");
#endif
#endif

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    int32_t q0_q30;
    int32_t q1_q30;
    int32_t q2_q30;
    int32_t q3_q30;
    uint16_t accuracy;
    uint8_t flags;
} imu_dmp_sample_t;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    int16_t temperature;

    uint16_t actual_rate_hz;

    uint8_t flags;
    uint8_t accel_range_code;
    uint8_t gyro_range_code;
} imu_raw_sample_t;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    int16_t values[AUX_PACKET_CHANNEL_COUNT];
} adc_sample_t;
