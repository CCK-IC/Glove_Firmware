#include "ble_packets.h"

#include <limits.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "system_state.h"
#include "app_config.h"

// _Static_assert(IMU_PACKET_Q30_SIZE == 28, "Unexpected Q30 packet size");
// _Static_assert(IMU_PACKET_COMPACT_SIZE == 20,
//                "Unexpected compact IMU packet size");
_Static_assert(AUX_PACKET_V1_SIZE == 6 + 8 * 2, "Unexpected AUX packet size");
_Static_assert(IMU_RAW_FULL_V1_SIZE == 36,
               "Unexpected full raw IMU packet size");
_Static_assert(IMU_RAW_COMPACT_V2_SIZE == 20,
               "Unexpected compact raw IMU packet size");
_Static_assert(STATUS_PACKET_FULL_SIZE == 64,
               "Unexpected full status packet size");
_Static_assert(STATUS_PACKET_COMPACT_SIZE == 20,
               "Unexpected compact status packet size");

static void put_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_i16_le(uint8_t *p, int16_t value)
{
    put_u16_le(p, (uint16_t)value);
}

static void put_u32_le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static int16_t q30_to_q14(int32_t value)
{
    /*
     * Q30 to Q14 is a 16-bit right shift. Apply symmetric rounding before
     * shifting and saturate to int16_t.
     */
    int64_t rounded = value;

    rounded = rounded >= 0 ? (rounded + (1LL << 15)) / (1LL << 16)
                           : -((-rounded + (1LL << 15)) / (1LL << 16));

    if (rounded > INT16_MAX) {
        rounded = INT16_MAX;
    }
    if (rounded < INT16_MIN) {
        rounded = INT16_MIN;
    }

    return (int16_t)rounded;
}

static uint8_t clamp_u8(uint32_t value)
{
    return value > UINT8_MAX ? UINT8_MAX : (uint8_t)value;
}

void ble_encode_imu_q30(
    uint8_t out[IMU_PACKET_Q30_SIZE],
    const imu_dmp_sample_t *sample)
{
    out[0] = IMU_PACKET_VERSION_Q30;
    out[1] = sample->flags;
    put_u16_le(out + 2, sample->accuracy);
    put_u32_le(out + 4, sample->sequence);
    put_u32_le(out + 8, sample->timestamp_ms);
    put_u32_le(out + 12, (uint32_t)sample->q0_q30);
    put_u32_le(out + 16, (uint32_t)sample->q1_q30);
    put_u32_le(out + 20, (uint32_t)sample->q2_q30);
    put_u32_le(out + 24, (uint32_t)sample->q3_q30);
}

void ble_encode_aux(uint8_t out[AUX_PACKET_V1_SIZE],
                    uint32_t ts, uint8_t sequence, const int16_t values[8])
{
    out[0] = AUX_PACKET_VERSION;
    out[1] = sequence;
    put_u32_le(out + 2, ts);
    for (int i = 0; i < 8; ++i) put_i16_le(out + 6 + i * 2, values[i]);
}

size_t ble_encode_aux_for_mtu(
    uint8_t *out, 
    size_t capacity, 
    uint16_t mtu,
    const adc_sample_t *sample)
{
    if (!out || !sample || mtu < 23) {
        ESP_LOGE("BLE PACKET", "Invalid parameters for aux packet encoding");
        return 0;
    }

    const size_t att_payload = (size_t)mtu - 3U;

    if (att_payload >= AUX_PACKET_V1_SIZE &&
        capacity >= AUX_PACKET_V1_SIZE) {
        ble_encode_aux(out, sample->timestamp_ms, sample->sequence, sample->values);
        return AUX_PACKET_V1_SIZE;
    }
    ESP_LOGW("BLE PACKET", "MTU too small for aux packet: mtu=%u, att_payload=%zu, capacity=%zu", mtu, att_payload, capacity);
    return 0;
}
void ble_encode_imu_compact(
    uint8_t out[IMU_PACKET_COMPACT_SIZE],
    const imu_dmp_sample_t *sample,
    uint8_t requested_rate_hz,
    uint8_t actual_rate_hz)
{
    /*
     * Version 2 compact packet:
     *
     *  0     u8     version = 2
     *  1     u8     flags
     *  2     u16    sequence, low 16 bits
     *  4     u32    timestamp_ms
     *  8     i16    q0 Q14
     * 10     i16    q1 Q14
     * 12     i16    q2 Q14
     * 14     i16    q3 Q14
     * 16     u16    accuracy
     * 18     u8     requested notification rate
     * 19     u8     measured sensor rate
     */
    out[0] = IMU_PACKET_VERSION_Q14_COMPACT;
    out[1] = sample->flags;
    put_u16_le(out + 2, (uint16_t)sample->sequence);
    put_u32_le(out + 4, sample->timestamp_ms);
    put_i16_le(out + 8, q30_to_q14(sample->q0_q30));
    put_i16_le(out + 10, q30_to_q14(sample->q1_q30));
    put_i16_le(out + 12, q30_to_q14(sample->q2_q30));
    put_i16_le(out + 14, q30_to_q14(sample->q3_q30));
    put_u16_le(out + 16, sample->accuracy);
    out[18] = requested_rate_hz;
    out[19] = actual_rate_hz;
}

size_t ble_encode_imu_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const imu_dmp_sample_t *sample)
{
    if (!out || !sample || mtu < 23) {
        return 0;
    }

    const size_t att_payload = (size_t)mtu - 3U;

    if (att_payload >= IMU_PACKET_Q30_SIZE &&
        capacity >= IMU_PACKET_Q30_SIZE) {
        ble_encode_imu_q30(out, sample);
        return IMU_PACKET_Q30_SIZE;
    }

#if CONFIG_BLE_COMPACT_PACKET_FALLBACK
    if (att_payload >= IMU_PACKET_COMPACT_SIZE &&
        capacity >= IMU_PACKET_COMPACT_SIZE) {
        system_state_t state;
        system_state_snapshot(&state);

        ble_encode_imu_compact(
            out,
            sample,
            clamp_u8(state.requested_rate_hz),
            clamp_u8(state.actual_dmp_sample_rate_hz));

        return IMU_PACKET_COMPACT_SIZE;
    }
#endif

    return 0;
}

void ble_encode_imu_raw_full(
    uint8_t packet[IMU_RAW_FULL_V1_SIZE],
    const imu_raw_sample_t *sample)
{
    packet[0] = IMU_RAW_PACKET_VERSION_FULL;
    packet[1] = sample->flags;
    packet[2] = sample->accel_range_code;
    packet[3] = sample->gyro_range_code;

    put_u32_le(packet + 4, sample->sequence);
    put_u32_le(packet + 8, sample->timestamp_ms);

    put_i16_le(packet + 12, sample->accel_x);
    put_i16_le(packet + 14, sample->accel_y);
    put_i16_le(packet + 16, sample->accel_z);

    put_i16_le(packet + 18, sample->gyro_x);
    put_i16_le(packet + 20, sample->gyro_y);
    put_i16_le(packet + 22, sample->gyro_z);

    put_i16_le(packet + 24, sample->mag_x);
    put_i16_le(packet + 26, sample->mag_y);
    put_i16_le(packet + 28, sample->mag_z);

    put_i16_le(packet + 30, sample->temperature);
    put_u16_le(packet + 32, sample->actual_rate_hz);
    put_u16_le(packet + 34, 0);

}

void ble_encode_imu_raw_compact(
    uint8_t packet[IMU_RAW_COMPACT_V2_SIZE],
    const imu_raw_sample_t *sample)
{
    uint8_t compact_flags = 0;

    if ((sample->flags & IMU_RAW_FLAG_VALID) != 0) {
        compact_flags |= 0x01U;
    }

    if ((sample->flags & IMU_RAW_FLAG_ACCEL_VALID) != 0) {
        compact_flags |= 0x02U;
    }

    if ((sample->flags & IMU_RAW_FLAG_GYRO_VALID) != 0) {
        compact_flags |= 0x04U;
    }

    if ((sample->flags & IMU_RAW_FLAG_DMP_CONCURRENT) != 0) {
        compact_flags |= 0x08U;
    }

    compact_flags |=
        (uint8_t)((sample->accel_range_code & 0x03U) << 4);

    compact_flags |=
        (uint8_t)((sample->gyro_range_code & 0x03U) << 6);

    packet[0] = IMU_RAW_PACKET_VERSION_COMPACT;
    packet[1] = compact_flags;

    put_u16_le(packet + 2, (uint16_t)sample->sequence);
    put_u32_le(packet + 4, sample->timestamp_ms);

    put_i16_le(packet + 8, sample->accel_x);
    put_i16_le(packet + 10, sample->accel_y);
    put_i16_le(packet + 12, sample->accel_z);

    put_i16_le(packet + 14, sample->gyro_x);
    put_i16_le(packet + 16, sample->gyro_y);
    put_i16_le(packet + 18, sample->gyro_z);

}

size_t ble_encode_imu_raw_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const imu_raw_sample_t *sample)
{
    if (!out || !sample || mtu < 23) {
        return 0;
    }

    const size_t att_payload = (size_t)mtu - 3U;

    if (att_payload >= IMU_RAW_FULL_V1_SIZE &&
        capacity >= IMU_RAW_FULL_V1_SIZE) {
        ble_encode_imu_raw_full(out, sample);
        return IMU_RAW_FULL_V1_SIZE;
    }

#if CONFIG_BLE_COMPACT_PACKET_FALLBACK
    if (att_payload >= IMU_RAW_COMPACT_V2_SIZE &&
        capacity >= IMU_RAW_COMPACT_V2_SIZE) {
        ble_encode_imu_raw_compact(out, sample);
        return IMU_RAW_COMPACT_V2_SIZE;
    }
#endif

    return 0;
}

void ble_encode_status_full(
    uint8_t out[STATUS_PACKET_FULL_SIZE])
{
    system_state_t state;
    system_state_snapshot(&state);

    memset(out, 0, STATUS_PACKET_FULL_SIZE);

    out[0] = STATUS_PACKET_VERSION_FULL;

#if CONFIG_TARGET_BOARD_XIAO_ESP32_C3
    out[1] = 1;
#else
    out[1] = 2;
#endif

    out[2] =
        (state.ble_connected ? 1U : 0U) |
        (state.advertising ? 2U : 0U) |
        (state.pairing ? 4U : 0U) |
        (state.encrypted ? 8U : 0U) |
        (state.bonded ? 16U : 0U) |
        (state.slow_mode ? 32U : 0U) |
        (state.battery_low ? 64U : 0U) |
        (state.imu_dmp ? 128U : 0U);

    out[3] = (uint8_t)state.imu_state;

    put_u32_le(out + 4,
               (uint32_t)(esp_timer_get_time() / 1000ULL));
    put_u16_le(out + 8, (uint16_t)state.requested_rate_hz);
    put_u16_le(out + 10, (uint16_t)state.actual_dmp_sample_rate_hz);
    put_u16_le(out + 12, (uint16_t)state.actual_notify_rate_hz);
    put_u16_le(out + 14, (uint16_t)state.status_period_ms);
    put_u16_le(out + 16, (uint16_t)state.battery_mv);

    out[18] = state.battery_percent;
    out[19] = state.who_am_i;

    put_u16_le(out + 20, state.mtu);
    put_u16_le(out + 22, state.conn_handle);
    put_u32_le(out + 24, esp_get_free_heap_size());
    put_u32_le(out + 28, esp_get_minimum_free_heap_size());
    put_u32_le(out + 32, state.sample_count);
    put_u32_le(out + 36, state.dropped_samples);
    put_u32_le(out + 40, state.dropped_notifications);
    put_u32_le(out + 44, state.fifo_overflows);
    put_u32_le(out + 48, state.i2c_errors);
    put_u32_le(out + 52, state.imu_reinitializations);
    put_u32_le(out + 56, (uint32_t)state.last_error);

    out[60] = APP_VERSION_MAJOR;
    out[61] = APP_VERSION_MINOR;
    out[62] = APP_VERSION_PATCH;
    out[63] = state.phy; /* Zero means unknown. */
}

void ble_encode_status_compact(
    uint8_t out[STATUS_PACKET_COMPACT_SIZE])
{
    system_state_t state;
    system_state_snapshot(&state);

    memset(out, 0, STATUS_PACKET_COMPACT_SIZE);

    out[0] = STATUS_PACKET_VERSION_COMPACT;
    out[1] =
        (state.ble_connected ? 1U : 0U) |
        (state.advertising ? 2U : 0U) |
        (state.pairing ? 4U : 0U) |
        (state.encrypted ? 8U : 0U) |
        (state.bonded ? 16U : 0U) |
        (state.slow_mode ? 32U : 0U) |
        (state.battery_low ? 64U : 0U) |
        (state.imu_dmp ? 128U : 0U);

    put_u32_le(out + 2,
               (uint32_t)(esp_timer_get_time() / 1000ULL));
    put_u16_le(out + 6, (uint16_t)state.requested_rate_hz);
    put_u16_le(out + 8, (uint16_t)state.actual_dmp_sample_rate_hz);
    put_u16_le(out + 10, (uint16_t)state.actual_notify_rate_hz);
    put_u16_le(out + 12, (uint16_t)state.battery_mv);

    out[14] = state.battery_percent;
    out[15] = (uint8_t)state.imu_state;

    put_u16_le(out + 16, state.mtu);

    /*
     * Compact counters are saturating low-resolution diagnostics.
     */
    out[18] = clamp_u8(state.dropped_notifications);
    out[19] = clamp_u8(state.i2c_errors + state.fifo_overflows);
}

size_t ble_encode_status_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu)
{
    if (!out || mtu < 23) {
        return 0;
    }

    const size_t att_payload = (size_t)mtu - 3U;

    if (att_payload >= STATUS_PACKET_FULL_SIZE &&
        capacity >= STATUS_PACKET_FULL_SIZE) {
        ble_encode_status_full(out);
        return STATUS_PACKET_FULL_SIZE;
    }

#if CONFIG_BLE_COMPACT_PACKET_FALLBACK
    if (att_payload >= STATUS_PACKET_COMPACT_SIZE &&
        capacity >= STATUS_PACKET_COMPACT_SIZE) {
        ble_encode_status_compact(out);
        return STATUS_PACKET_COMPACT_SIZE;
    }
#endif

    return 0;
}