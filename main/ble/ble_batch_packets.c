#include "ble_batch_packets.h"

#include <string.h>
#include "esp_system.h"
#include "esp_timer.h"

static void u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void u32(uint8_t *p, uint32_t v)
{
    u16(p, (uint16_t)v);
    u16(p + 2, (uint16_t)(v >> 16));
}

static size_t slot_size(unsigned kind)
{
    switch (kind) {
    case BLE_BATCH_KIND_RAW: return 24;
    case BLE_BATCH_KIND_DMP: return 22;
    case BLE_BATCH_KIND_AUX: return 20;
    default: return 0;
    }
}

size_t ble_batch_frame_size(unsigned kind, size_t count)
{
    if (!count || count > BLE_BATCH_MAX_SAMPLES) return 0;
    if (kind == BLE_BATCH_KIND_STATUS)
        return count == 1 ? BLE_BATCH_STATUS_SIZE : 0;
    size_t slot = slot_size(kind);
    return slot ? BLE_BATCH_HEADER_SIZE + count * 2 * slot : 0;
}

static size_t value_capacity(uint16_t mtu)
{
    if (mtu < 23 || mtu > 517) return 0;
    size_t capacity = mtu - 3U;
    return capacity < BLE_BATCH_MAX_FRAME_SIZE ? capacity : BLE_BATCH_MAX_FRAME_SIZE;
}

size_t ble_batch_sample_limit(unsigned kind, uint16_t mtu)
{
    size_t capacity = value_capacity(mtu);
    if (!capacity) return 0;
    if (kind == BLE_BATCH_KIND_STATUS) return 1;
    size_t slot = slot_size(kind);
    if (!slot) return 0;
    if (capacity < BLE_BATCH_HEADER_SIZE + 2 * slot) return BLE_BATCH_MAX_SAMPLES;
    size_t count = (capacity - BLE_BATCH_HEADER_SIZE) / (2 * slot);
    return count < BLE_BATCH_MAX_SAMPLES ? count : BLE_BATCH_MAX_SAMPLES;
}

static bool deltas(uint32_t seq, uint32_t time, uint32_t base_seq,
                   uint32_t base_time, uint32_t prev_seq, uint32_t prev_time,
                   size_t index)
{
    uint32_t ds = seq - base_seq;
    uint32_t dt = time - base_time;
    return ds <= UINT8_MAX && dt <= UINT16_MAX &&
        (!index || (ds > prev_seq - base_seq && dt >= prev_time - base_time));
}

static void header(uint8_t *frame, unsigned kind, size_t count,
                   uint32_t seq, uint32_t time, uint16_t rate, uint8_t format)
{
    frame[0] = 3;
    frame[1] = (uint8_t)kind;
    frame[2] = 1;
    frame[3] = (uint8_t)count;
    u32(frame + 4, seq);
    u32(frame + 8, time);
    u16(frame + 12, rate);
    frame[14] = format;
}

static void prefix(uint8_t *p, uint32_t seq, uint32_t time,
                   uint32_t base_seq, uint32_t base_time, uint8_t flags)
{
    p[0] = (uint8_t)(seq - base_seq);
    u16(p + 1, (uint16_t)(time - base_time));
    p[3] = flags;
}

size_t ble_batch_encode_raw(uint8_t *out, size_t capacity,
                           const imu_raw_sample_t *samples, size_t count)
{
    size_t length = ble_batch_frame_size(BLE_BATCH_KIND_RAW, count);
    if (!out || !samples || !length || capacity < length) return 0;
    const imu_raw_sample_t *base = samples;
    for (size_t i = 0; i < count; ++i) {
        const imu_raw_sample_t *s = samples + i;
        const imu_raw_sample_t *prev = samples + (i ? i - 1 : 0);
        if ((s->flags & ~0x7fU) || s->accel_range_code > 3 || s->gyro_range_code > 3 ||
            s->actual_rate_hz != base->actual_rate_hz ||
            s->accel_range_code != base->accel_range_code ||
            s->gyro_range_code != base->gyro_range_code ||
            !deltas(s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms,
                    prev->sequence, prev->timestamp_ms, i)) return 0;
    }
    uint8_t frame[BLE_BATCH_MAX_FRAME_SIZE] = {0};
    header(frame, BLE_BATCH_KIND_RAW, count, base->sequence, base->timestamp_ms,
           base->actual_rate_hz, base->accel_range_code | (base->gyro_range_code << 2));
    for (size_t i = 0; i < count; ++i) {
        const imu_raw_sample_t *s = samples + i;
        uint8_t *p = frame + BLE_BATCH_HEADER_SIZE + i * 48;
        prefix(p, s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms, s->flags);
        const int16_t values[] = {s->accel_x, s->accel_y, s->accel_z,
            s->gyro_x, s->gyro_y, s->gyro_z, s->mag_x, s->mag_y, s->mag_z, s->temperature};
        for (size_t n = 0; n < 10; ++n) u16(p + 4 + n * 2, (uint16_t)values[n]);
    }
    memcpy(out, frame, length);
    return length;
}

size_t ble_batch_encode_dmp(uint8_t *out, size_t capacity,
                           const imu_dmp_sample_t *samples, size_t count,
                           uint16_t measured_rate_hz)
{
    size_t length = ble_batch_frame_size(BLE_BATCH_KIND_DMP, count);
    if (!out || !samples || !length || capacity < length) return 0;
    const imu_dmp_sample_t *base = samples;
    for (size_t i = 0; i < count; ++i) {
        const imu_dmp_sample_t *s = samples + i;
        const imu_dmp_sample_t *prev = samples + (i ? i - 1 : 0);
        if ((s->flags & ~0x1fU) ||
            !deltas(s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms,
                    prev->sequence, prev->timestamp_ms, i)) return 0;
    }
    uint8_t frame[BLE_BATCH_MAX_FRAME_SIZE] = {0};
    header(frame, BLE_BATCH_KIND_DMP, count, base->sequence, base->timestamp_ms,
           measured_rate_hz, 0);
    for (size_t i = 0; i < count; ++i) {
        const imu_dmp_sample_t *s = samples + i;
        uint8_t *p = frame + BLE_BATCH_HEADER_SIZE + i * 44;
        prefix(p, s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms, s->flags);
        u32(p + 4, (uint32_t)s->q0_q30);
        u32(p + 8, (uint32_t)s->q1_q30);
        u32(p + 12, (uint32_t)s->q2_q30);
        u32(p + 16, (uint32_t)s->q3_q30);
        u16(p + 20, s->accuracy);
    }
    memcpy(out, frame, length);
    return length;
}

size_t ble_batch_encode_aux(uint8_t *out, size_t capacity,
                           const adc_sample_t *samples, size_t count,
                           uint16_t measured_rate_hz)
{
    size_t length = ble_batch_frame_size(BLE_BATCH_KIND_AUX, count);
    if (!out || !samples || !length || capacity < length ||
        APP_AUX_ADC_CHANNEL_COUNT < 1 || APP_AUX_ADC_CHANNEL_COUNT > 8) return 0;
    const adc_sample_t *base = samples;
    for (size_t i = 0; i < count; ++i) {
        const adc_sample_t *s = samples + i;
        const adc_sample_t *prev = samples + (i ? i - 1 : 0);
        if (!deltas(s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms,
                    prev->sequence, prev->timestamp_ms, i)) return 0;
    }
    uint8_t frame[BLE_BATCH_MAX_FRAME_SIZE] = {0};
    header(frame, BLE_BATCH_KIND_AUX, count, base->sequence, base->timestamp_ms,
           measured_rate_hz, APP_AUX_ADC_CHANNEL_COUNT);
    for (size_t i = 0; i < count; ++i) {
        const adc_sample_t *s = samples + i;
        uint8_t *p = frame + BLE_BATCH_HEADER_SIZE + i * 40;
        prefix(p, s->sequence, s->timestamp_ms, base->sequence, base->timestamp_ms, 1);
#if APP_AUX_ADC_CHANNEL_COUNT > 0
        for (size_t n = 0; n < APP_AUX_ADC_CHANNEL_COUNT; ++n)
            u16(p + 4 + n * 2, (uint16_t)s->values[n]);
#endif
    }
    memcpy(out, frame, length);
    return length;
}

size_t ble_batch_encode_status(uint8_t *out, size_t capacity, const system_state_t *state)
{
    if (!out || !state || capacity < BLE_BATCH_STATUS_SIZE) return 0;
    uint8_t frame[BLE_BATCH_STATUS_SIZE] = {3, BLE_BATCH_KIND_STATUS, 1, 1};
    uint8_t *p = frame + 4;
    u32(p, (uint32_t)((uint64_t)esp_timer_get_time() / 1000U));
    u16(p + 4, (uint16_t)state->requested_imu_rate_hz);
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
    u16(p + 6, (uint16_t)state->actual_raw_sample_rate_hz);
#else
    u16(p + 6, (uint16_t)state->actual_dmp_sample_rate_hz);
#endif
    u16(p + 8, (uint16_t)state->actual_imu_notify_rate_hz);
    u16(p + 10, (uint16_t)state->battery_mv);
    u16(p + 12, state->mtu);
#if CONFIG_TARGET_BOARD_XIAO_ESP32_C3
    p[14] = 1;
#else
    p[14] = 2;
#endif
    p[15] = (uint8_t)state->imu_state;
    p[16] = state->battery_percent;
    p[17] = (state->ble_connected ? 1U : 0U) | (state->advertising ? 2U : 0U) |
        (state->pairing ? 4U : 0U) | (state->encrypted ? 8U : 0U) |
        (state->bonded ? 16U : 0U) | (state->slow_mode ? 32U : 0U) |
        (state->battery_low ? 64U : 0U) | (state->imu_dmp ? 128U : 0U);
    p[18] = APP_VERSION_MAJOR;
    p[19] = APP_VERSION_MINOR;
    p[20] = APP_VERSION_PATCH;
    p[21] = state->phy;
    u16(p + 22, 3);
    u32(p + 24, esp_get_free_heap_size());
    u32(p + 28, state->dropped_notifications);
    u32(p + 32, state->fifo_overflows);
    u32(p + 36, state->i2c_errors);
    u32(p + 40, state->dropped_sensor_samples);
    u32(p + 44, state->raw_sample_count);
    u32(p + 48, (uint32_t)state->last_error);
    memcpy(out, frame, sizeof(frame));
    return sizeof(frame);
}

unsigned ble_batch_fragment_count(size_t frame_length, uint16_t mtu)
{
    size_t capacity = value_capacity(mtu);
    if (!capacity || !frame_length || frame_length > BLE_BATCH_MAX_FRAME_SIZE) return 0;
    if (frame_length <= capacity) return 1;
    size_t payload = capacity - BLE_BATCH_FRAGMENT_HEADER_SIZE;
    return (unsigned)((frame_length + payload - 1) / payload);
}

size_t ble_batch_fragment(uint8_t *out, size_t capacity,
                         const uint8_t *frame, size_t frame_length, uint16_t mtu,
                         uint16_t frame_id, unsigned index)
{
    unsigned count = ble_batch_fragment_count(frame_length, mtu);
    if (!out || !frame || !count || index >= count || frame_length < 4 ||
        frame[0] != 3 || frame[2] != 1 ||
        ble_batch_frame_size(frame[1], frame[3]) != frame_length) return 0;
    if (count == 1) {
        if (capacity < frame_length) return 0;
        memmove(out, frame, frame_length);
        return frame_length;
    }
    size_t payload = value_capacity(mtu) - BLE_BATCH_FRAGMENT_HEADER_SIZE;
    size_t offset = index * payload;
    size_t length = frame_length - offset;
    if (length > payload) length = payload;
    if (capacity < length + BLE_BATCH_FRAGMENT_HEADER_SIZE) return 0;
    uint8_t fragment[BLE_BATCH_MAX_FRAME_SIZE] = {0xf3, frame[1]};
    u16(fragment + 2, frame_id);
    fragment[4] = (uint8_t)index;
    fragment[5] = (uint8_t)count;
    u16(fragment + 6, (uint16_t)frame_length);
    memcpy(fragment + BLE_BATCH_FRAGMENT_HEADER_SIZE, frame + offset, length);
    memcpy(out, fragment, length + BLE_BATCH_FRAGMENT_HEADER_SIZE);
    return length + BLE_BATCH_FRAGMENT_HEADER_SIZE;
}
