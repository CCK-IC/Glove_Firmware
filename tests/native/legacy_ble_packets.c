#include "legacy_ble_packets.h"

#include <limits.h>

enum {
    IMU_PACKET_VERSION_Q14_COMPACT = 2,
    AUX_PACKET_VERSION = 1,
};

_Static_assert(IMU_PACKET_COMPACT_SIZE == 20, "Unexpected compact IMU packet size");
_Static_assert(AUX_PACKET_V1_SIZE == 6 + AUX_PACKET_CHANNEL_COUNT * 2,
               "Unexpected AUX packet size");

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

void ble_encode_aux(uint8_t out[AUX_PACKET_V1_SIZE],
                    uint32_t timestamp_ms, uint8_t sequence,
                    const int16_t values[AUX_PACKET_CHANNEL_COUNT])
{
    out[0] = AUX_PACKET_VERSION;
    out[1] = sequence;
    put_u32_le(out + 2, timestamp_ms);
    const unsigned active_channels = APP_AUX_ADC_CHANNEL_COUNT;
    for (unsigned i = 0; i < AUX_PACKET_CHANNEL_COUNT; ++i) {
        const int16_t value = i < active_channels ? values[i] : 0;
        put_i16_le(out + 6 + i * 2, value);
    }
}

size_t ble_encode_aux_for_mtu(
    uint8_t *out,
    size_t capacity,
    uint16_t mtu,
    const adc_sample_t *sample)
{
    if (!out || !sample || mtu < 23) {
        return 0;
    }

    const size_t att_payload = (size_t)mtu - 3U;

    if (att_payload >= AUX_PACKET_V1_SIZE &&
        capacity >= AUX_PACKET_V1_SIZE) {
        ble_encode_aux(out, sample->timestamp_ms, sample->sequence, sample->values);
        return AUX_PACKET_V1_SIZE;
    }
    return 0;
}

void ble_encode_imu_compact(
    uint8_t out[IMU_PACKET_COMPACT_SIZE],
    const imu_dmp_sample_t *sample,
    uint8_t requested_imu_rate_hz,
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
    out[18] = requested_imu_rate_hz;
    out[19] = actual_rate_hz;
}
