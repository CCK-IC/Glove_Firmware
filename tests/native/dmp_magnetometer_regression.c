/* Include the actual manager to exercise its static FIFO consumer and raw
 * publication path without extracting or duplicating production logic. Driver
 * headers/types and magnetometer cache functions are the production versions.
 * The FIFO fake advances wall time inside calls, including terminal/error reads.
 */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "../../main/imu/imu_manager.c"

typedef struct {
    icm20948_status_e status;
    icm_20948_DMP_data_t data;
    int64_t duration;
} frame_t;
static frame_t frames[16];
static unsigned queued, consumed, fifo_reads, resets, reports, agmt_reads, raw_aux_calls;
static int64_t clock_us;
static icm20948_status_e reset_result, agmt_result;
static system_state_t shared;

int64_t esp_timer_get_time(void) { return clock_us; }
void esp_rom_delay_us(uint32_t us) { clock_us += us; }
void system_state_increment_counter(size_t offset) {
    assert(offset + sizeof(uint32_t) <= sizeof(shared));
    uint32_t value;
    memcpy(&value, (char *)&shared + offset, sizeof(value));
    ++value;
    memcpy((char *)&shared + offset, &value, sizeof(value));
}
void system_state_write_field(size_t offset, const void *value, size_t size) {
    assert(offset + size <= sizeof(shared));
    memcpy((char *)&shared + offset, value, size);
}
void system_state_set_error(const char *module, int32_t error) {
    assert(!strcmp(module, "imu_fifo") && error == ESP_FAIL);
    ++reports;
}
icm20948_status_e inv_icm20948_read_dmp_data(icm20948_device_t *device, icm_20948_DMP_data_t *data) {
    assert(device == &s_sensor);
    ++fifo_reads;
    if (consumed == queued) return ICM_20948_STAT_FIFO_NO_DATA_AVAIL;
    frame_t *frame = &frames[consumed++];
    clock_us += frame->duration;
    *data = frame->data;
    return frame->status;
}
icm20948_status_e icm20948_reset_fifo(icm20948_device_t *device) {
    assert(device == &s_sensor);
    assert(device->_last_bank == 255);
    ++resets;
    /* A failed bank-select write can leave a plausible but incorrect cache.
     * Every recovery retry must invalidate it again before reaching here. */
    device->_last_bank = reset_result == ICM_20948_STAT_OK ? 0 : 3;
    if (reset_result == ICM_20948_STAT_OK) queued = consumed;
    return reset_result;
}
icm20948_status_e icm20948_get_agmt(icm20948_device_t *device, icm20948_agmt_t *data) {
    assert(device == &s_sensor);
    ++agmt_reads;
    memset(data, 0, sizeof(*data));
    data->acc.axes.x = 123;
    data->gyr.axes.y = -456;
    data->mag.axes.x = 999; /* Deliberately incompatible DMP shadow layout. */
    if (agmt_result != ICM_20948_STAT_OK) device->_last_bank = 3;
    return agmt_result;
}
/* Any raw auxiliary transaction in DMP mode is a harness failure. */
icm20948_status_e icm20948_set_bank(icm20948_device_t *d, uint8_t b) {
    (void)d; (void)b; ++raw_aux_calls; assert(!"raw SLV4 in DMP mode"); return ICM_20948_STAT_ERR;
}
icm20948_status_e icm20948_execute_r(icm20948_device_t *d, uint8_t r, uint8_t *v, uint32_t n) {
    (void)d; (void)r; (void)v; (void)n; assert(0); return ICM_20948_STAT_ERR;
}
icm20948_status_e icm20948_execute_w(icm20948_device_t *d, uint8_t r, uint8_t *v, uint32_t n) {
    (void)d; (void)r; (void)v; (void)n; assert(0); return ICM_20948_STAT_ERR;
}
static void reset_test(void) {
    memset(frames, 0, sizeof(frames));
    memset(&s_magnetometer, 0, sizeof(s_magnetometer));
    memset(&s_latest_raw_sample, 0, sizeof(s_latest_raw_sample));
    memset(&shared, 0, sizeof(shared));
    queued = consumed = fifo_reads = resets = reports = agmt_reads = raw_aux_calls = 0;
    clock_us = 1000000;
    reset_result = ICM_20948_STAT_OK;
    agmt_result = ICM_20948_STAT_OK;
    s_sensor_ready = true;
    s_sensor._last_bank = 3;
    s_dmp_fifo_needs_reset = false;
    s_dmp_sequence = s_raw_sequence = 0;
}
static frame_t *push(icm20948_status_e status, uint16_t header, int x, int64_t duration) {
    assert(queued < 16);
    frame_t *frame = &frames[queued++];
    frame->status = status;
    frame->duration = duration;
    frame->data.header = header;
    frame->data.Compass.Data.X = (int16_t)x;
    frame->data.Compass.Data.Y = INT16_MIN;
    frame->data.Compass.Data.Z = INT16_MAX;
    frame->data.Quat9.Data.Q1 = x;
    frame->data.Quat9.Data.Accuracy = 42;
    return frame;
}
static void test_independent_outputs(void) {
    reset_test();
    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass, -123, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(s_magnetometer.valid && s_magnetometer.x == -123);
    assert(s_magnetometer.y == INT16_MIN && s_magnetometer.z == INT16_MAX);
    uint64_t stamp = s_magnetometer.updated_at_us;
    clock_us += 10000;
    assert(!read_dmp_sample(&sample, &errors));
    assert(s_magnetometer.updated_at_us == stamp);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Quat9, 17, 100);
    assert(read_dmp_sample(&sample, &errors));
    assert(sample.q1_q30 == 17 && sample.accuracy == 42 && sample.sequence == 1);
    assert(s_magnetometer.updated_at_us == stamp && s_magnetometer.x == -123);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass_Calibr, 999, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(s_magnetometer.x == -123 && s_magnetometer.updated_at_us == stamp);
    system_state_t state = {.imu_dmp = true, .actual_raw_sample_rate_hz = 100};
    assert(acquire_raw_sample(clock_us, &state));
    assert(agmt_reads == 1 && raw_aux_calls == 0);
    assert(s_latest_raw_sample.mag_x == -123 && s_latest_raw_sample.accel_x == 123);
    assert(s_latest_raw_sample.flags & IMU_RAW_FLAG_DMP_CONCURRENT);
    assert(s_latest_raw_sample.flags & IMU_RAW_FLAG_MAG_VALID);
    clock_us = (int64_t)stamp + IMU_MAG_FRESHNESS_US;
    assert(acquire_raw_sample(clock_us, &state));
    assert(!(s_latest_raw_sample.flags & IMU_RAW_FLAG_MAG_VALID));
}
static void test_agmt_failure(void) {
    for (unsigned dmp = 0; dmp < 2; ++dmp) {
        reset_test();
        imu_magnetometer_update(&s_magnetometer, -12, 34, -56, clock_us);
        system_state_t state = {.imu_dmp = true, .actual_raw_sample_rate_hz = 100};
        assert(acquire_raw_sample(clock_us, &state));
        assert(s_latest_raw_sample.flags & IMU_RAW_FLAG_MAG_VALID);
        unsigned char published[sizeof(s_latest_raw_sample)];
        memcpy(published, &s_latest_raw_sample, sizeof(published));
        uint32_t sequence = s_raw_sequence;
        agmt_result = ICM_20948_STAT_ERR;
        state.imu_dmp = dmp != 0;
        assert(!acquire_raw_sample(clock_us, &state));
        assert(s_sensor._last_bank == 255);
        assert(!s_magnetometer.valid && !s_magnetometer.overflow);
        assert(shared.dropped_raw_samples == 1 && shared.raw_sample_count == 1);
        assert(s_raw_sequence == sequence);
        assert(!memcmp(published, &s_latest_raw_sample, sizeof(published)));
        assert(agmt_reads == 2 && !raw_aux_calls && !fifo_reads && !resets);
    }
}
static void test_latest_and_abort(void) {
    reset_test();
    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Quat9, 7, 100);
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Compass, 8, 100);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass | DMP_header_bitmap_Quat9, 9, 100);
    assert(read_dmp_sample(&sample, &errors));
    assert(consumed == 3 && !resets && sample.q1_q30 == 9 && s_magnetometer.x == 9);
    reset_test();
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Compass, 8, 100);
    push(ICM_20948_STAT_FIFO_INCOMPLETE_DATA, 0, 0, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(!s_magnetometer.valid && resets == 1 && !s_dmp_sequence);
    reset_test();
    for (int i = 0; i < 9; ++i)
        push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Compass, i, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(consumed == 8 && resets == 1 && !s_magnetometer.valid);
    reset_test();
    for (int i = 0; i < 8; ++i)
        push(i == 7 ? ICM_20948_STAT_OK : ICM_20948_STAT_FIFO_MORE_DATA_AVAIL,
             DMP_header_bitmap_Compass, i, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(consumed == 8 && !resets && s_magnetometer.valid && s_magnetometer.x == 7);
    reset_test();
    for (int i = 0; i < 3; ++i)
        push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Compass, i, 2500);
    assert(!read_dmp_sample(&sample, &errors));
    assert(consumed == 2 && resets == 1 && !s_magnetometer.valid);
    reset_test();
    reset_result = ICM_20948_STAT_ERR;
    push(ICM_20948_STAT_FIFO_INCOMPLETE_DATA, 0, 0, 100);
    assert(!read_dmp_sample(&sample, &errors));
    assert(!s_magnetometer.valid && reports == 1);
    reset_test();
    imu_magnetometer_update(&s_magnetometer, 1, 2, 3, clock_us);
    assert(imu_manager_reset_fifo() == ESP_OK && !s_magnetometer.valid);
}
static void test_terminal_deadline(void) {
    const int64_t durations[] = {4999, 5000, 20000};
    for (unsigned i = 0; i < sizeof(durations) / sizeof(durations[0]); ++i) {
        reset_test();
        unsigned errors = 0;
        imu_dmp_sample_t sample = {0};
        imu_magnetometer_update(&s_magnetometer, 7, 8, 9, clock_us);
        push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass | DMP_header_bitmap_Quat9,
             1, durations[i]);
        bool published = read_dmp_sample(&sample, &errors);
        if (durations[i] < 5000) {
            assert(published && s_magnetometer.valid && !resets);
            assert(sample.q1_q30 == 1 && s_magnetometer.x == 1);
        } else {
            assert(!published && !s_magnetometer.valid && resets == 1);
            assert(!s_dmp_sequence && s_magnetometer.x == 7);
        }
    }
}
static void test_transport_error_recovery(void) {
    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    reset_test();
    imu_magnetometer_update(&s_magnetometer, 7, 8, 9, clock_us);
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL,
         DMP_header_bitmap_Compass | DMP_header_bitmap_Quat9, 99, 100);
    push(ICM_20948_STAT_ERR, 0, 0, 100); /* May have consumed header/payload. */
    assert(!read_dmp_sample(&sample, &errors));
    assert(resets == 1 && !s_magnetometer.valid && !s_dmp_fifo_needs_reset);
    assert(s_magnetometer.x == 7 && !s_dmp_sequence && consumed == 2);
}
static void test_reset_gate(void) {
    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    reset_test();
    reset_result = ICM_20948_STAT_ERR;
    push(ICM_20948_STAT_FIFO_INCOMPLETE_DATA, 0, 0, 100);
    assert(!read_dmp_sample(&sample, &errors) && resets == 1 && reports == 1);
    assert(s_dmp_fifo_needs_reset);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass | DMP_header_bitmap_Quat9, 1, 100);
    unsigned reads_before = fifo_reads;
    for (unsigned retry = 0; retry < 3; ++retry) {
        assert(!read_dmp_sample(&sample, &errors));
        assert(fifo_reads == reads_before && consumed == 1);
        assert(s_dmp_fifo_needs_reset && !s_magnetometer.valid && !s_dmp_sequence);
    }
    assert(resets == 4 && reports == 4);
    reset_result = ICM_20948_STAT_OK;
    assert(!read_dmp_sample(&sample, &errors)); /* Reset drops the unread frame. */
    assert(resets == 5 && !s_dmp_fifo_needs_reset && !s_magnetometer.valid);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Compass | DMP_header_bitmap_Quat9, 123, 100);
    assert(read_dmp_sample(&sample, &errors));
    assert(s_magnetometer.valid && s_magnetometer.x == 123 && sample.sequence == 1);
    /* Manual reset failure must use the same read gate. */
    reset_result = ICM_20948_STAT_ERR;
    assert(imu_manager_reset_fifo() == ESP_FAIL);
    assert(s_dmp_fifo_needs_reset && !s_magnetometer.valid);
    reads_before = fifo_reads;
    assert(!read_dmp_sample(&sample, &errors) && fifo_reads == reads_before);
    reset_result = ICM_20948_STAT_OK;
    assert(imu_manager_reset_fifo() == ESP_OK && !s_dmp_fifo_needs_reset);
}
int main(void) {
    test_independent_outputs();
    test_agmt_failure();
    test_latest_and_abort();
    puts("PASS: Compass-only, quaternion-only, calibrated isolation, native counts, raw DMP publication, freshness boundary");
    puts("PASS: latest frame, incomplete rollback, frame/time backlog caps, reset failure, explicit reset invalidation (ASan/UBSan)");
    test_terminal_deadline();
    test_transport_error_recovery();
    test_reset_gate();
    puts("PASS: unconditional terminal deadline (4999/5000/20000us), destructive error rollback, failed-reset read gate, recovery, manual reset gate");
    puts("PASS: bank cache invalidated before every reset attempt, including retries after cache-poisoning failures");
    puts("PASS: AGMT failure in raw/DMP modes invalidates magnetic and bank caches, increments dropped count, preserves published sample and sequence");
    return 0;
}
