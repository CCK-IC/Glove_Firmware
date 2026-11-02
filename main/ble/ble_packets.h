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


enum {
    IMU_PACKET_VERSION_Q30 = 1,
    IMU_PACKET_VERSION_Q14_COMPACT = 2,
    IMU_RAW_PACKET_VERSION_FULL = 1,
    IMU_RAW_PACKET_VERSION_COMPACT = 2,
    STATUS_PACKET_VERSION_FULL = 1,
    STATUS_PACKET_VERSION_COMPACT = 2,
    AUX_PACKET_VERSION = 1,
};

#define IMU_PACKET_Q30_SIZE       28U
#define IMU_PACKET_COMPACT_SIZE   20U
#define IMU_RAW_FULL_V1_SIZE      36U
#define IMU_RAW_COMPACT_V2_SIZE   20U
#define STATUS_PACKET_FULL_SIZE   64U
#define STATUS_PACKET_COMPACT_SIZE 20U
#define AUX_PACKET_V1_SIZE 22U


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
    int16_t values[8];
    // int16_t adc1;
    // int16_t adc2;
    // int16_t adc3;
    // int16_t adc4;
    // int16_t adc5;
    // int16_t adc6;
    // int16_t adc7;
    // int16_t adc8;
} adc_sample_t;

// // void ble_encode_imu(uint8_t out[APP_IMU_PACKET_SIZE],
// //                     const imu_dmp_sample_t *sample);
size_t ble_encode_aux_for_mtu(uint8_t *out, 
    size_t capacity, 
    uint16_t mtu,
    const adc_sample_t *sample);

void ble_encode_aux(uint8_t out[AUX_PACKET_V1_SIZE],
    uint32_t timestamp_ms, uint8_t sequence,
    const int16_t values[8]);
void ble_encode_status(uint8_t out[APP_STATUS_SIZE]);

void ble_encode_imu_q30(
    uint8_t out[IMU_PACKET_Q30_SIZE],
    const imu_dmp_sample_t *sample);

void ble_encode_imu_compact(
    uint8_t out[IMU_PACKET_COMPACT_SIZE],
    const imu_dmp_sample_t *sample,
    uint8_t requested_rate_hz,
    uint8_t actual_rate_hz);

size_t ble_encode_imu_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const imu_dmp_sample_t *sample);

void ble_encode_imu_raw_full(
    uint8_t out[IMU_RAW_FULL_V1_SIZE],
    const imu_raw_sample_t *sample);

void ble_encode_imu_raw_compact(
    uint8_t out[IMU_RAW_COMPACT_V2_SIZE],
    const imu_raw_sample_t *sample);

size_t ble_encode_imu_raw_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const imu_raw_sample_t *sample);


void ble_encode_status_full(
    uint8_t out[STATUS_PACKET_FULL_SIZE]);

void ble_encode_status_compact(
    uint8_t out[STATUS_PACKET_COMPACT_SIZE]);

size_t ble_encode_status_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu);