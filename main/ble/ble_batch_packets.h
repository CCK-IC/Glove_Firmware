#pragma once

#include "ble_packets.h"
#include "system_state.h"

#define BLE_BATCH_MAX_SAMPLES 8U
#define BLE_BATCH_MAX_FRAME_SIZE 512U
#define BLE_BATCH_HEADER_SIZE 28U
#define BLE_BATCH_STATUS_SIZE 108U
#define BLE_BATCH_FRAGMENT_HEADER_SIZE 8U

typedef enum {
    BLE_BATCH_KIND_RAW = 1,
    BLE_BATCH_KIND_DMP = 2,
    BLE_BATCH_KIND_AUX = 3,
    BLE_BATCH_KIND_STATUS = 4,
} ble_batch_kind_t;

/* Return zero for invalid arguments; encoders never modify out on failure.
 * Samples must have strictly increasing representable sequence deltas and
 * nondecreasing representable timestamp deltas, including across u32 wrap.
 * Only the local slot is present; the physical peer slot is always zero.
 */
size_t ble_batch_frame_size(unsigned kind, size_t count);
/* If no row fits, return eight rows for subsequent application fragmentation.
 * Status always has one row. ATT MTUs outside 23..517 are invalid.
 */
size_t ble_batch_sample_limit(unsigned kind, uint16_t mtu);
size_t ble_batch_encode_raw(uint8_t *out, size_t capacity,
    const imu_raw_sample_t *samples, size_t count);
size_t ble_batch_encode_dmp(uint8_t *out, size_t capacity,
    const imu_dmp_sample_t *samples, size_t count, uint16_t measured_rate_hz);
size_t ble_batch_encode_aux(uint8_t *out, size_t capacity,
    const adc_sample_t *samples, size_t count, uint16_t measured_rate_hz);
size_t ble_batch_encode_status(uint8_t *out, size_t capacity,
    const system_state_t *state);
unsigned ble_batch_fragment_count(size_t frame_length, uint16_t mtu);
/* A fitting frame is returned unchanged at index zero (no fragment header).
 * Fragmentation accepts complete v3 frames produced by the encoders above.
 */
size_t ble_batch_fragment(uint8_t *out, size_t capacity,
    const uint8_t *frame, size_t frame_length, uint16_t mtu,
    uint16_t frame_id, unsigned index);
