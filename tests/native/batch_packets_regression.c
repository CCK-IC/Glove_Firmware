/* Production codec checks; no ESP-IDF/NimBLE or production-state mutations. */
#include "ble/ble_batch_packets.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned timer_calls, heap_calls;
int64_t esp_timer_get_time(void) { ++timer_calls; return INT64_C(4294967301000); }
uint32_t esp_get_free_heap_size(void) { ++heap_calls; return UINT32_C(0x12345678); }

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)le16(p) | (uint32_t)le16(p + 2) << 16; }
static void zeros(const uint8_t *p, size_t length) {
    for (size_t i = 0; i < length; ++i) assert(p[i] == 0);
}

static imu_raw_sample_t raw[8];
static imu_dmp_sample_t dmp[8];
static adc_sample_t aux[8];
static const int16_t readings[10] = {INT16_MIN, INT16_MAX, -1, 0, 1, -1234, 2345, -32767, 32766, -99};
static const int32_t quaternion[4] = {1073741824, -1073741824, INT32_MIN, INT32_MAX};

static void init_samples(void) {
    for (unsigned i = 0; i < 8; ++i) {
        /* Wrap both bases while preserving irregular actual intervals and gaps. */
        uint32_t sequence = UINT32_MAX - 2 + i * 2;
        uint32_t timestamp = UINT32_MAX - 9 + i * i * 3;
        raw[i] = (imu_raw_sample_t){.sequence=sequence, .timestamp_ms=timestamp,
            .actual_rate_hz=123, .flags=0x7f, .accel_range_code=2, .gyro_range_code=3,
            .accel_x=readings[0], .accel_y=readings[1], .accel_z=readings[2],
            .gyro_x=readings[3], .gyro_y=readings[4], .gyro_z=readings[5],
            .mag_x=readings[6], .mag_y=readings[7], .mag_z=readings[8], .temperature=readings[9]};
        dmp[i] = (imu_dmp_sample_t){.sequence=sequence, .timestamp_ms=timestamp,
            .q0_q30=quaternion[0], .q1_q30=quaternion[1], .q2_q30=quaternion[2],
            .q3_q30=quaternion[3], .accuracy=UINT16_MAX, .flags=0x1f};
        aux[i] = (adc_sample_t){.sequence=sequence, .timestamp_ms=timestamp};
        for (unsigned j = 0; j < 8; ++j) aux[i].values[j] = readings[j];
    }
}

static size_t encode(unsigned kind, uint8_t *out, size_t capacity, size_t count) {
    switch (kind) {
    case BLE_BATCH_KIND_RAW: return ble_batch_encode_raw(out, capacity, raw, count);
    case BLE_BATCH_KIND_DMP: return ble_batch_encode_dmp(out, capacity, dmp, count, 456);
    case BLE_BATCH_KIND_AUX: return ble_batch_encode_aux(out, capacity, aux, count, 789);
    default: abort();
    }
}

static void test_layouts(void) {
    const size_t slots[] = {0, 24, 22, 20};
    for (unsigned kind = 1; kind <= 3; ++kind) {
        if (kind == 3 && APP_AUX_ADC_CHANNEL_COUNT == 0) continue;
        for (size_t count = 1; count <= 8; ++count) {
            uint8_t guarded[514];
            memset(guarded, 0xa5, sizeof(guarded));
            uint8_t *out = guarded + 1;
            size_t length = 28 + count * 2 * slots[kind];
            assert(encode(kind, out, length, count) == length);
            assert(guarded[0] == 0xa5);
            for (size_t i = length + 1; i < sizeof(guarded); ++i) assert(guarded[i] == 0xa5);
            assert(out[0] == 3 && out[1] == kind && out[2] == 1 && out[3] == count);
            assert(le32(out + 4) == UINT32_MAX - 2);
            assert(le32(out + 8) == UINT32_MAX - 9);
            assert(le16(out + 12) == (kind == 1 ? 123 : kind == 2 ? 456 : 789));
            assert(out[14] == (kind == 1 ? 14 : kind == 2 ? 0 : APP_AUX_ADC_CHANNEL_COUNT));
            zeros(out + 15, 13);
            for (size_t i = 0; i < count; ++i) {
                const uint8_t *p = out + 28 + i * 2 * slots[kind];
                assert(p[0] == i * 2 && le16(p + 1) == i * i * 3);
                assert((uint32_t)(le32(out + 4) + p[0]) == raw[i].sequence);
                assert((uint32_t)(le32(out + 8) + le16(p + 1)) == raw[i].timestamp_ms);
                assert(p[3] == (kind == 1 ? 0x7f : kind == 2 ? 0x1f : 1));
                zeros(p + slots[kind], slots[kind]);
                if (kind == 1) {
                    for (unsigned j = 0; j < 10; ++j) assert(le16(p + 4 + j * 2) == (uint16_t)readings[j]);
                } else if (kind == 2) {
                    for (unsigned j = 0; j < 4; ++j) assert(le32(p + 4 + j * 4) == (uint32_t)quaternion[j]);
                    assert(le16(p + 20) == UINT16_MAX);
                } else {
                    for (unsigned j = 0; j < 8; ++j)
                        assert(le16(p + 4 + j * 2) == (j < APP_AUX_ADC_CHANNEL_COUNT ? (uint16_t)readings[j] : 0));
                }
            }
        }
    }
}

static uint8_t invalid_out[520];
#define REJECT(call) do { \
    memset(invalid_out, 0x5a, sizeof(invalid_out)); \
    assert((call) == 0); \
    for (size_t reject_i = 0; reject_i < sizeof(invalid_out); ++reject_i) assert(invalid_out[reject_i] == 0x5a); \
} while (0)

static void test_invalid(void) {
    for (unsigned kind = 1; kind <= 3; ++kind) {
        REJECT(encode(kind, invalid_out, sizeof(invalid_out), 0));
        REJECT(encode(kind, invalid_out, sizeof(invalid_out), 9));
        REJECT(encode(kind, invalid_out, sizeof(invalid_out), SIZE_MAX));
        for (size_t cap = 0; cap < ble_batch_frame_size(kind, 2); ++cap)
            REJECT(encode(kind, invalid_out, cap, 2));
        assert(!encode(kind, NULL, 512, 2));
    }
    REJECT(ble_batch_encode_raw(invalid_out, 512, NULL, 2));
    REJECT(ble_batch_encode_dmp(invalid_out, 512, NULL, 2, 0));
    REJECT(ble_batch_encode_aux(invalid_out, 512, NULL, 2, 0));
    /* Change the LAST entry: prove all validation precedes any output write. */
    raw[7].flags = 0x80;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    raw[7].accel_range_code = 4;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    raw[7].gyro_range_code = 4;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    raw[7].accel_range_code = 1;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    raw[7].gyro_range_code = 2;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    raw[7].actual_rate_hz++;
    REJECT(encode(1, invalid_out, 512, 8)); init_samples();
    dmp[7].flags = 0x20;
    REJECT(encode(2, invalid_out, 512, 8)); init_samples();

    /* Identical checks on all three streams: max deltas, ordering, time wrap. */
    for (unsigned kind = 1; kind <= 3; ++kind) {
        if (kind == 3 && APP_AUX_ADC_CHANNEL_COUNT == 0) continue;
#define SET_SECOND(field, value) do { raw[1].field = (value); dmp[1].field = (value); aux[1].field = (value); } while (0)
        SET_SECOND(sequence, raw[0].sequence + 255U);
        SET_SECOND(timestamp_ms, raw[0].timestamp_ms + 65535U);
        assert(encode(kind, invalid_out, 512, 2));
        SET_SECOND(sequence, raw[0].sequence + 256U);
        REJECT(encode(kind, invalid_out, 512, 2));
        SET_SECOND(sequence, raw[0].sequence);
        REJECT(encode(kind, invalid_out, 512, 2));
        SET_SECOND(sequence, raw[0].sequence - 1U);
        REJECT(encode(kind, invalid_out, 512, 2));
        SET_SECOND(sequence, raw[0].sequence + 1U);
        SET_SECOND(timestamp_ms, raw[0].timestamp_ms + 65536U);
        REJECT(encode(kind, invalid_out, 512, 2));
        SET_SECOND(timestamp_ms, raw[0].timestamp_ms - 1U);
        REJECT(encode(kind, invalid_out, 512, 2));
        SET_SECOND(timestamp_ms, raw[0].timestamp_ms);
        assert(encode(kind, invalid_out, 512, 2)); /* equal timestamps allowed */
        init_samples();
        SET_SECOND(timestamp_ms, raw[0].timestamp_ms + 100U);
        REJECT(encode(kind, invalid_out, 512, 3)); /* third timestamp goes backwards */
        init_samples();
        SET_SECOND(sequence, raw[0].sequence + 100U);
        REJECT(encode(kind, invalid_out, 512, 3)); /* third sequence goes backwards */
        init_samples();
    }
    if (APP_AUX_ADC_CHANNEL_COUNT == 0) REJECT(encode(3, invalid_out, 512, 1));
}

static system_state_t status_state(void) {
    return (system_state_t){.requested_imu_rate_hz=200, .actual_raw_sample_rate_hz=123,
        .actual_dmp_sample_rate_hz=456, .actual_imu_notify_rate_hz=42, .battery_mv=4200,
        .mtu=247, .imu_state=IMU_STATE_DEGRADED, .battery_percent=87,
        .ble_connected=true, .advertising=true, .pairing=true, .encrypted=true,
        .bonded=true, .slow_mode=true, .battery_low=true, .imu_dmp=true, .phy=2,
        .dropped_notifications=UINT32_C(0x89abcdef), .fifo_overflows=UINT32_MAX,
        .i2c_errors=UINT32_C(0x12345678), .dropped_sensor_samples=UINT32_C(0xfedcba98),
        .raw_sample_count=UINT32_C(0x76543210), .last_error=INT32_MIN};
}

static void test_status(void) {
    system_state_t state = status_state(), copy;
    memcpy(&copy, &state, sizeof(state));
    uint8_t out[110]; memset(out, 0xa5, sizeof(out));
    assert(ble_batch_encode_status(out + 1, 108, &state) == 108);
    assert(!memcmp(&state, &copy, sizeof(state)));
    assert(out[0] == 0xa5 && out[109] == 0xa5);
    assert(!memcmp(out + 1, (uint8_t[]){3,4,1,1}, 4));
    const uint8_t *p = out + 5;
    assert(le32(p) == 5); /* timer's ms wraps u32 */
    assert(le16(p + 4) == 200);
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
    assert(le16(p + 6) == 123);
#else
    assert(le16(p + 6) == 456);
#endif
    assert(le16(p + 8) == 42 && le16(p + 10) == 4200 && le16(p + 12) == 247);
    assert(p[14] == (CONFIG_TARGET_BOARD_XIAO_ESP32_C3 ? 1 : 2));
    assert(p[15] == IMU_STATE_DEGRADED && p[16] == 87 && p[17] == 255);
    assert(p[18] == APP_VERSION_MAJOR && p[19] == APP_VERSION_MINOR && p[20] == APP_VERSION_PATCH);
    assert(p[21] == 2 && le16(p + 22) == 3);
    assert(le32(p + 24) == UINT32_C(0x12345678));
    assert(le32(p + 28) == state.dropped_notifications);
    assert(le32(p + 32) == state.fifo_overflows && le32(p + 36) == state.i2c_errors);
    assert(le32(p + 40) == state.dropped_sensor_samples && le32(p + 44) == state.raw_sample_count);
    assert(le32(p + 48) == (uint32_t)INT32_MIN);
    zeros(p + 52, 52);
    unsigned tc = timer_calls, hc = heap_calls;
    for (size_t cap = 0; cap < BLE_BATCH_STATUS_SIZE; ++cap)
        REJECT(ble_batch_encode_status(invalid_out, cap, &state));
    REJECT(ble_batch_encode_status(invalid_out, 512, NULL));
    assert(!ble_batch_encode_status(NULL, 512, &state));
    assert(timer_calls == tc && heap_calls == hc);
}

static void test_limits(void) {
    const uint16_t mtus[] = {23, 128, 247, 517};
    const size_t expected[][3] = {{8,8,8}, {2,2,2}, {4,4,5}, {8,8,8}};
    for (size_t m = 0; m < 4; ++m) {
        for (unsigned k = 1; k <= 3; ++k) assert(ble_batch_sample_limit(k, mtus[m]) == expected[m][k-1]);
        assert(ble_batch_sample_limit(4, mtus[m]) == 1);
    }
    for (unsigned k = 1; k <= 3; ++k) {
        size_t row = ble_batch_frame_size(k, 1);
        assert(ble_batch_sample_limit(k, (uint16_t)(row + 2)) == 8);
        assert(ble_batch_sample_limit(k, (uint16_t)(row + 3)) == 1);
    }
    assert(!ble_batch_frame_size(0, 1) && !ble_batch_frame_size(5, 1));
    assert(!ble_batch_frame_size(4, 2) && ble_batch_frame_size(4, 1) == 108);
    assert(!ble_batch_frame_size(1, SIZE_MAX));
    assert(!ble_batch_sample_limit(0, 247) && !ble_batch_sample_limit(5, 247));
    assert(!ble_batch_sample_limit(1, 22) && !ble_batch_sample_limit(1, 518));
    assert(!ble_batch_fragment_count(0, 23) && !ble_batch_fragment_count(513, 517));
    assert(!ble_batch_fragment_count(SIZE_MAX, 23));
    assert(!ble_batch_fragment_count(108, 22) && !ble_batch_fragment_count(108, 65535));
    assert(ble_batch_fragment_count(512, 517) == 1);
    assert(ble_batch_fragment_count(512, 23) == 43);
}

static void test_fragments(void) {
    const uint16_t mtus[] = {23, 24, 75, 76, 128, 247, 517};
    for (unsigned kind = 1; kind <= 4; ++kind) {
        if (kind == 3 && APP_AUX_ADC_CHANNEL_COUNT == 0) continue;
        for (size_t rows = 1; rows <= (kind == 4 ? 1U : 8U); ++rows) {
            uint8_t frame[512];
            system_state_t state = status_state();
            size_t length = kind == 4 ? ble_batch_encode_status(frame, 512, &state) : encode(kind, frame, 512, rows);
            assert(length);
            for (size_t m = 0; m < sizeof(mtus)/sizeof(mtus[0]); ++m) {
                uint16_t mtu = mtus[m];
                size_t cap = mtu - 3U; if (cap > 512) cap = 512;
                unsigned count = ble_batch_fragment_count(length, mtu);
                assert(count == (length <= cap ? 1 : (length + cap - 9) / (cap - 8)));
                uint8_t joined[512]; size_t used = 0;
                for (unsigned i = 0; i < count; ++i) {
                    uint8_t guarded[514]; memset(guarded, 0xa5, sizeof(guarded));
                    uint8_t *out = guarded + 1;
                    size_t n = ble_batch_fragment(out, cap, frame, length, mtu, UINT16_MAX, i);
                    assert(n && n <= cap && guarded[0] == 0xa5);
                    for (size_t j = n + 1; j < sizeof(guarded); ++j) assert(guarded[j] == 0xa5);
                    for (size_t short_cap = 0; short_cap < n; ++short_cap)
                        REJECT(ble_batch_fragment(invalid_out, short_cap, frame, length, mtu, 0, i));
                    if (count == 1) { assert(n == length && !memcmp(out, frame, n)); memcpy(joined, out, n); used = n; }
                    else {
                        assert(out[0] == 0xf3 && out[1] == kind && le16(out + 2) == UINT16_MAX);
                        assert(out[4] == i && out[5] == count && le16(out + 6) == length);
                        if (i + 1 < count) assert(n == cap);
                        assert(n > 8 && used + n - 8 <= sizeof(joined));
                        memcpy(joined + used, out + 8, n - 8); used += n - 8;
                    }
                }
                assert(used == length && !memcmp(joined, frame, length));
                REJECT(ble_batch_fragment(invalid_out, 512, frame, length, mtu, 0, count));
                REJECT(ble_batch_fragment(invalid_out, 512, frame, length, mtu, 0, UINT_MAX));
            }
            uint8_t copy[512]; memcpy(copy, frame, length);
            size_t n = ble_batch_fragment(copy, 512, copy, length, 517, 0, 0);
            assert(n == length && !memcmp(copy, frame, length));
            memcpy(copy, frame, length);
            assert(ble_batch_fragment(copy, 512, copy, length, 23, 0, 0) == 20);
            assert(copy[2] == 0 && copy[3] == 0); /* frame ID wrap to zero */
            REJECT(ble_batch_fragment(invalid_out, 512, NULL, length, 23, 0, 0));
            assert(!ble_batch_fragment(NULL, 512, frame, length, 23, 0, 0));
            frame[0] = 2; REJECT(ble_batch_fragment(invalid_out, 512, frame, length, 23, 0, 0)); frame[0] = 3;
            frame[1] = 5; REJECT(ble_batch_fragment(invalid_out, 512, frame, length, 23, 0, 0)); frame[1] = (uint8_t)kind;
            frame[2] = 3; REJECT(ble_batch_fragment(invalid_out, 512, frame, length, 23, 0, 0)); frame[2] = 1;
            frame[3] = 0; REJECT(ble_batch_fragment(invalid_out, 512, frame, length, 23, 0, 0)); frame[3] = (uint8_t)rows;
            REJECT(ble_batch_fragment(invalid_out, 512, frame, length - 1, 23, 0, 0));
            REJECT(ble_batch_fragment(invalid_out, 512, frame, 3, 23, 0, 0));
            REJECT(ble_batch_fragment(invalid_out, 512, frame, 513, 23, 0, 0));
            REJECT(ble_batch_fragment(invalid_out, 512, frame, length, 22, 0, 0));
        }
    }
}

static void json_bytes(FILE *f, const uint8_t *bytes, size_t n) {
    fputc('[', f);
    for (size_t i = 0; i < n; ++i) fprintf(f, "%s%u", i ? "," : "", bytes[i]);
    fputc(']', f);
}

static void fixtures(const char *path) {
    FILE *f = fopen(path, "w"); assert(f);
    fprintf(f, "{\n  \"protocol\":3,\n  \"generator\":\"production C ble_batch_packets codec\",\n  \"active_aux_channels\":%u,\n  \"cases\":[\n", (unsigned)APP_AUX_ADC_CHANNEL_COUNT);
    for (unsigned kind = 1; kind <= 4; ++kind) {
        uint8_t frame[512]; system_state_t state = status_state();
        size_t length = kind == 4 ? ble_batch_encode_status(frame, 512, &state) : encode(kind, frame, 512, 3);
        assert(length);
        fprintf(f, "    %s{\"name\":\"%s\",\"kind\":%u,\"frame\":", kind == 1 ? "" : ",", kind == 1 ? "raw_wrap_signed" : kind == 2 ? "dmp_wrap_signed" : kind == 3 ? "aux_wrap_inactive_zero" : "status_signed_error", kind);
        json_bytes(f, frame, length);
        if (kind <= 3) {
            fprintf(f, ",\"samples\":[");
            for (unsigned i = 0; i < 3; ++i) {
                fprintf(f, "%s{\"gloveId\":0,\"sequence\":%lu,\"timestampMs\":%lu,\"flags\":%u", i ? "," : "", (unsigned long)raw[i].sequence, (unsigned long)raw[i].timestamp_ms, kind == 1 ? 127 : kind == 2 ? 31 : 1);
                if (kind == 1) {
                    fprintf(f, ",\"rateHz\":123,\"accelRange\":2,\"gyroRange\":3,\"values\":[");
                    for (unsigned j = 0; j < 10; ++j) fprintf(f, "%s%d", j ? "," : "", readings[j]);
                    fputc(']', f);
                } else if (kind == 2) fprintf(f, ",\"rateHz\":456,\"q30\":[1073741824,-1073741824,-2147483648,2147483647],\"accuracy\":65535");
                else {
                    fprintf(f, ",\"rateHz\":789,\"values\":[");
                    for (unsigned j = 0; j < 8; ++j) fprintf(f, "%s%d", j ? "," : "", j < APP_AUX_ADC_CHANNEL_COUNT ? readings[j] : 0);
                    fputc(']', f);
                }
                fputc('}', f);
            }
            fputc(']', f);
        } else {
            fprintf(f, ",\"expected\":{\"gloveId\":0,\"uptimeMs\":5,\"requestedRateHz\":200,\"actualRateHz\":123,\"notificationRateHz\":42,\"batteryMv\":4200,\"mtu\":247,\"boardId\":1,\"imuState\":3,\"batteryPercent\":87,\"flags\":255,\"firmware\":[%u,%u,%u],\"phy\":2,\"featureFlags\":3,\"freeHeapBytes\":305419896,\"droppedNotifications\":2309737967,\"fifoOverflows\":4294967295,\"i2cErrors\":305419896,\"droppedSensorSamples\":4275878552,\"rawSampleCount\":1985229328,\"lastError\":-2147483648}", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);
        }
        fprintf(f, ",\"frameId\":65535,\"fragmentsMtu23\":[");
        unsigned count = ble_batch_fragment_count(length, 23);
        for (unsigned i = 0; i < count; ++i) {
            uint8_t part[512]; size_t n = ble_batch_fragment(part, sizeof(part), frame, length, 23, UINT16_MAX, i);
            assert(n); if (i) fputc(',', f); json_bytes(f, part, n);
        }
        fprintf(f, "]}\n");
    }
    fprintf(f, "  ]\n}\n"); assert(!ferror(f)); assert(!fclose(f));
}

int main(int argc, char **argv) {
    init_samples();
    test_limits(); test_layouts(); test_invalid(); test_status(); test_fragments();
    if (argc == 2) fixtures(argv[1]); else assert(argc == 1);
    puts("PASS: v3 paired layouts, signed/wrap/delta limits, zero peer/inactive slots, immutable status, atomic rejection, MTU/fragment round trips (ASan/UBSan)");
    return 0;
}
