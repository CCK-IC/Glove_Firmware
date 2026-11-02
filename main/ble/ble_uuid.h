#pragma once
#include "host/ble_uuid.h"

/*
 * UUID strings:
 * IMU service:    8f7c0001-4f54-4d55-8000-5849414f494d
 * Quaternion:     8f7c0002-4f54-4d55-8000-5849414f494d
 * Raw IMU:        8f7c0099-4f54-4d55-8000-5849414f494d
 * Aux service:    8f7c0010-4f54-4d55-8000-5849414f494d
 * Aux data:       8f7c0011-4f54-4d55-8000-5849414f494d
 * Status service: 8f7c0020-4f54-4d55-8000-5849414f494d
 * Status data:    8f7c0021-4f54-4d55-8000-5849414f494d
 * Control:        8f7c0022-4f54-4d55-8000-5849414f494d
 *
 * BLE_UUID128_INIT takes bytes in little-endian wire order.
 */
#define XIAO_UUID128(suffix) \
    BLE_UUID128_INIT(0x4d,0x49,0x4f,0x41,0x49,0x58,0x00,0x80, \
                     0x55,0x4d,0x54,0x4f,(suffix),0x00,0x7c,0x8f)

extern const ble_uuid128_t UUID_IMU_SERVICE;
extern const ble_uuid128_t UUID_IMU_QUAT;
extern const ble_uuid128_t UUID_IMU_RAW;
// extern const ble_uuid128_t UUID_AUX_SERVICE;
// extern const ble_uuid128_t UUID_AUX_DATA;
extern const ble_uuid128_t UUID_STATUS_SERVICE;
extern const ble_uuid128_t UUID_STATUS_DATA;
extern const ble_uuid128_t UUID_CONTROL;