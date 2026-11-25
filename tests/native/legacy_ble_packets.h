#pragma once

#include "ble/ble_packets.h"

#define IMU_PACKET_COMPACT_SIZE 20U
#define AUX_PACKET_V1_SIZE 22U

void ble_encode_aux(uint8_t out[AUX_PACKET_V1_SIZE],
    uint32_t timestamp_ms, uint8_t sequence,
    const int16_t values[AUX_PACKET_CHANNEL_COUNT]);

size_t ble_encode_aux_for_mtu(uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const adc_sample_t *sample);

void ble_encode_imu_compact(
    uint8_t out[IMU_PACKET_COMPACT_SIZE],
    const imu_dmp_sample_t *sample,
    uint8_t requested_imu_rate_hz,
    uint8_t actual_rate_hz);
