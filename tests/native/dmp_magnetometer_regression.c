/* Include the actual manager to exercise its static FIFO consumer and raw
 * publication path without extracting or duplicating production logic. Driver
 * headers/types and magnetometer cache functions are the production versions.
 * The FIFO fake advances wall time inside calls, including terminal/error reads.
 */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#define imu_magnetometer_configure fake_magnetometer_configure
#include "../../main/imu/imu_manager.c"
#undef imu_magnetometer_configure

typedef struct {
    icm20948_status_e status;
    icm_20948_DMP_data_t data;
    int64_t duration;
    esp_err_t transport_error;
} frame_t;
static frame_t frames[16];
static unsigned queued, consumed, fifo_reads, resets, reports, agmt_reads, raw_aux_calls;
static int64_t clock_us;
static icm20948_status_e reset_result, agmt_result, sample_mode_result;
static esp_err_t agmt_transport_error;
static bool runtime_sensor_asleep, raw_aux_nack;
static unsigned bus_resets, sensor_resets, transport_attaches, transport_begins;
static unsigned device_removals, device_additions;
static uint32_t added_clock_hz;
static int scl_level, sda_level;
static system_state_t shared;
static uint32_t buffer_epochs[4];
static unsigned buffered_raw_count, buffered_dmp_count;
uint32_t ble_sample_buffer_epoch(unsigned kind) { return buffer_epochs[kind]; }
void ble_sample_buffer_invalidate(unsigned kind) { ++buffer_epochs[kind]; }
void ble_sample_buffer_publish_raw(const imu_raw_sample_t *sample, uint32_t epoch) {
    (void)sample;
    if (epoch == buffer_epochs[BLE_BUFFER_RAW]) ++buffered_raw_count;
}
void ble_sample_buffer_publish_dmp(const imu_dmp_sample_t *sample, uint32_t epoch) {
    (void)sample;
    if (epoch == buffer_epochs[BLE_BUFFER_DMP]) ++buffered_dmp_count;
}

int64_t esp_timer_get_time(void) { return clock_us; }
void vTaskDelay(TickType_t ticks) { clock_us += ticks * 1000LL; }
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
    assert(!strcmp(module, "imu_fifo") || !strcmp(module, "imu"));
    shared.last_error = error;
    ++reports;
}
i2c_master_bus_handle_t board_i2c_bus(void) { return (void *)1; }
void board_i2c_line_levels(int *scl, int *sda) { *scl = scl_level; *sda = sda_level; }
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus) {
    assert(bus == board_i2c_bus());
    ++bus_resets;
    return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
                                   const i2c_device_config_t *config,
                                   i2c_master_dev_handle_t *device) {
    assert(bus == board_i2c_bus() && config->device_address == CONFIG_IMU_I2C_ADDRESS);
    assert(!s_i2c_config.dev_handle);
    ++device_additions;
    added_clock_hz = config->scl_speed_hz;
    *device = (void *)3;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device) {
    assert(device && device == s_i2c_config.dev_handle);
    ++device_removals;
    return ESP_OK;
}
void icm20948_init_i2c(icm20948_device_t *device, icm0948_config_i2c_t *config) {
    assert(device == &s_sensor && config == &s_i2c_config && config->dev_handle);
    device->_last_bank = 255;
    device->_last_mems_bank = 255;
}
icm20948_status_e icm20948_sw_reset(icm20948_device_t *device) {
    assert(device == &s_sensor);
    ++sensor_resets;
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_sleep(icm20948_device_t *device, bool enabled) {
    assert(device == &s_sensor && !enabled);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_low_power(icm20948_device_t *device, bool enabled) {
    assert(device == &s_sensor && !enabled);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *device,
    icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode) {
    assert(device == &s_sensor && sensors && mode == SAMPLE_MODE_CONTINUOUS);
    return sample_mode_result;
}
icm20948_status_e icm20948_set_full_scale(icm20948_device_t *device,
    icm20948_internal_sensor_id_bm sensors, icm20948_fss_t scale) {
    assert(device == &s_sensor && sensors && scale.a == GPM_2 && scale.g == DPS_250);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_set_dlpf_cfg(icm20948_device_t *device,
    icm20948_internal_sensor_id_bm sensors, icm20948_dlpcfg_t filter) {
    assert(device == &s_sensor && sensors && filter.a == ACC_D473BW_N499BW);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_enable_dlpf(icm20948_device_t *device,
    icm20948_internal_sensor_id_bm sensors, bool enabled) {
    assert(device == &s_sensor && sensors && !enabled);
    return ICM_20948_STAT_OK;
}
icm20948_status_e fake_magnetometer_configure(icm20948_device_t *device,
                                           imu_magnetometer_t *magnetometer, bool dmp) {
    assert(device == &s_sensor && magnetometer == &s_magnetometer && dmp);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_init_dmp_sensor_with_defaults(icm20948_device_t *device) {
    assert(device == &s_sensor);
    return ICM_20948_STAT_OK;
}
icm20948_status_e inv_icm20948_enable_dmp_sensor(icm20948_device_t *device,
                                              enum inv_icm20948_sensor sensor, int enabled) {
    assert(device == &s_sensor && enabled == 1);
    assert(sensor == INV_ICM20948_SENSOR_ORIENTATION ||
           sensor == INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
    return ICM_20948_STAT_OK;
}
icm20948_status_e inv_icm20948_set_dmp_sensor_period(icm20948_device_t *device,
                                                   enum DMP_ODR_Registers register_id,
                                                   uint16_t interval) {
    assert(device == &s_sensor && !interval);
    assert(register_id == DMP_ODR_Reg_Quat9 || register_id == DMP_ODR_Reg_Cpass);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_enable_fifo(icm20948_device_t *device, bool enabled) {
    assert(device == &s_sensor && enabled);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_enable_dmp(icm20948_device_t *device, bool enabled) {
    assert(device == &s_sensor && enabled);
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_reset_dmp(icm20948_device_t *device) {
    assert(device == &s_sensor);
    return ICM_20948_STAT_OK;
}
void imu_i2c_attach(imu_i2c_transport_t *transport, icm20948_device_t *device) {
    assert(transport == &s_i2c && device == &s_sensor && transport->device);
    ++transport_attaches;
    transport->first_error = ESP_OK;
}
void imu_i2c_begin(imu_i2c_transport_t *transport, int timeout_ms) {
    assert(s_runtime_recovery_armed && transport == &s_i2c && timeout_ms == IMU_STREAM_TIMEOUT_MS);
    ++transport_begins;
    transport->first_error = ESP_OK;
}
icm20948_status_e icm20948_get_who_am_i(icm20948_device_t *device, uint8_t *identity) {
    assert(device == &s_sensor);
    assert(device->_last_bank == 255 && device->_last_mems_bank == 255);
    *identity = ICM_20948_WHOAMI;
    device->_last_bank = 0;
    return ICM_20948_STAT_OK;
}
const char *esp_err_to_name(esp_err_t error) { (void)error; return "test error"; }
icm20948_status_e inv_icm20948_read_dmp_data(icm20948_device_t *device, icm_20948_DMP_data_t *data) {
    assert(device == &s_sensor);
    ++fifo_reads;
    if (consumed == queued) return ICM_20948_STAT_FIFO_NO_DATA_AVAIL;
    frame_t *frame = &frames[consumed++];
    clock_us += frame->duration;
    *data = frame->data;
    s_i2c.first_error = frame->transport_error;
    return frame->status;
}
icm20948_status_e icm20948_reset_fifo(icm20948_device_t *device) {
    assert(device == &s_sensor);
    assert(device->_last_bank == 255 ||
           ((!s_sensor_ready || s_runtime_recovery_armed) && device->_last_bank == 0));
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
    if (s_runtime_recovery_armed) s_i2c.first_error = agmt_transport_error;
    return agmt_result;
}
/* The raw-mode regression fails at the first auxiliary status read. */
icm20948_status_e icm20948_set_bank(icm20948_device_t *d, uint8_t b) {
    assert(d == &s_sensor && raw_aux_nack && b == 0);
    ++raw_aux_calls;
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_execute_r(icm20948_device_t *d, uint8_t r, uint8_t *v, uint32_t n) {
    assert(s_runtime_recovery_armed && d == &s_sensor && n == 1);
    if (r == AGB0_REG_I2C_MST_STATUS) {
        assert(raw_aux_nack);
        s_i2c.first_error = ESP_ERR_INVALID_RESPONSE;
        s_i2c.failed_register = r;
        return ICM_20948_STAT_ERR;
    }
    assert(r == AGB0_REG_PWR_MGMT_1);
    *v = runtime_sensor_asleep ? 0x41 : 0x01;
    return ICM_20948_STAT_OK;
}
icm20948_status_e icm20948_execute_w(icm20948_device_t *d, uint8_t r, uint8_t *v, uint32_t n) {
    (void)d; (void)r; (void)v; (void)n; assert(0); return ICM_20948_STAT_ERR;
}
static void reset_test(void) {
    memset(frames, 0, sizeof(frames));
    memset(&s_magnetometer, 0, sizeof(s_magnetometer));
    memset(&s_latest_raw_sample, 0, sizeof(s_latest_raw_sample));
    memset(&shared, 0, sizeof(shared));
    memset(buffer_epochs, 0, sizeof(buffer_epochs));
    buffered_raw_count = buffered_dmp_count = 0;
    s_dmp_accepted_count = 0;
    queued = consumed = fifo_reads = resets = reports = agmt_reads = raw_aux_calls = 0;
    clock_us = 1000000;
    reset_result = ICM_20948_STAT_OK;
    agmt_result = ICM_20948_STAT_OK;
    sample_mode_result = ICM_20948_STAT_OK;
    agmt_transport_error = ESP_OK;
    runtime_sensor_asleep = raw_aux_nack = false;
    s_runtime_recovery_armed = false;
    s_i2c_config.dev_handle = (void *)2;
    memset(&s_i2c, 0, sizeof(s_i2c));
    bus_resets = sensor_resets = transport_attaches = transport_begins = 0;
    device_removals = device_additions = 0;
    added_clock_hz = 0;
    scl_level = sda_level = 1;
    s_sensor_ready = true;
    s_raw_read_failures = 0;
    s_dmp_reset_failures = 0;
    s_recovery_due_us = 0;
    s_recovery_delay_ms = 0;
    s_recovery_sample_deadline_us = 0;
    s_startup_retries_left = IMU_STARTUP_RETRY_COUNT;
    s_reconfigure_on_retry = false;
    s_i2c_clock_hz = CONFIG_BOARD_I2C_FREQUENCY_HZ;
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
static void test_preserved_fifo_batch(void) {
    reset_test();
    unsigned errors = 0;
    imu_dmp_sample_t sample;
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Quat9, 7, 1000);
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Quat9, 8, 1000);
    push(ICM_20948_STAT_OK, DMP_header_bitmap_Quat9, 9, 1000);
    assert(read_dmp_sample(&sample, &errors));
    assert(s_dmp_accepted_count == 3 && sample.sequence == 3);
    for (unsigned i = 0; i < 3; ++i) {
        assert(s_dmp_accepted[i].sequence == i + 1);
        assert(s_dmp_accepted[i].q1_q30 == 7 + (int)i);
        assert(s_dmp_accepted[i].timestamp_ms == 1001 + i);
        publish_dmp_sample(&s_dmp_accepted[i], buffer_epochs[BLE_BUFFER_DMP]);
    }
    assert(buffered_dmp_count == 3 && shared.dmp_sample_count == 3);
    assert(s_latest_dmp_sample.sequence == 3);
    reset_test();
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Quat9, 1, 1000);
    push(ICM_20948_STAT_ERR, 0, 0, 1000);
    assert(!read_dmp_sample(&sample, &errors));
    assert(s_dmp_accepted_count == 0 && s_dmp_sequence == 0 && buffered_dmp_count == 0);
}

static void test_repeated_raw_failures_stop_acquisition(void) {
    reset_test();
    system_state_t state = {.imu_dmp = true};
    assert(acquire_raw_sample(clock_us, &state));
    agmt_result = ICM_20948_STAT_ERR;
    agmt_transport_error = ESP_ERR_INVALID_RESPONSE;
    for (unsigned i = 0; i < CONFIG_IMU_ERROR_REINIT_COUNT; ++i)
        assert(!acquire_raw_sample(clock_us, &state));
    imu_raw_sample_t latest;
    assert(!imu_manager_latest_raw(&latest));
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(shared.dropped_raw_samples == CONFIG_IMU_ERROR_REINIT_COUNT);
    assert(shared.i2c_errors == CONFIG_IMU_ERROR_REINIT_COUNT);
    assert(s_recovery_due_us == clock_us + IMU_RECOVERY_FIRST_MS * 1000LL);
    const unsigned reads_before = agmt_reads;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(agmt_reads == reads_before);
    assert(shared.dropped_raw_samples == CONFIG_IMU_ERROR_REINIT_COUNT);
}

static void test_timeout_schedules_fast_recovery(void) {
    reset_test();
    system_state_t state = {.ble_connected = true, .imu_dmp = true,
                            .requested_imu_rate_hz = 90};
    shared = state;
    assert(acquire_raw_sample(clock_us, &state));
    agmt_result = ICM_20948_STAT_ERR;
    agmt_transport_error = ESP_ERR_TIMEOUT;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(!s_sensor_ready && shared.i2c_errors == 1 && shared.last_error == ESP_ERR_TIMEOUT);
    assert(s_recovery_due_us == clock_us + IMU_RECOVERY_FIRST_MS * 1000LL);
    assert(s_i2c.device && !bus_resets);
    assert(shared.ble_connected && shared.requested_imu_rate_hz == 90);

    sda_level = 0;
    assert(reset_i2c_bus() == ESP_ERR_TIMEOUT);
    assert(bus_resets == 1 && s_i2c.device == s_i2c_config.dev_handle);
    sda_level = 1;
    assert(reset_i2c_bus() == ESP_OK);
    assert(verify_runtime_sensor() == ESP_OK && resets == 1);
    runtime_sensor_asleep = true;
    assert(verify_runtime_sensor() == ESP_ERR_INVALID_STATE && resets == 1);
    assert(s_i2c.device == s_i2c_config.dev_handle);

    reset_test();
    confirm_hardware_sample();
    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    push(ICM_20948_STAT_FIFO_MORE_DATA_AVAIL, DMP_header_bitmap_Quat9, 1, 100);
    push(ICM_20948_STAT_ERR, 0, 0, 100)->transport_error = ESP_ERR_TIMEOUT;
    assert(!read_dmp_sample(&sample, &errors));
    assert(!s_sensor_ready && shared.last_error == ESP_ERR_TIMEOUT);
    assert(!resets && !s_dmp_accepted_count && shared.i2c_errors == 1);
}

static void test_recovery_only_arms_after_first_sample(void) {
    reset_test();
    const system_state_t state = {.imu_dmp = true};
    agmt_result = ICM_20948_STAT_ERR;
    for (unsigned i = 0; i < CONFIG_IMU_ERROR_REINIT_COUNT; ++i)
        assert(!acquire_raw_sample(clock_us, &state));
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(!s_runtime_recovery_armed);
    assert(s_recovery_due_us == clock_us + IMU_STARTUP_RETRY_MS * 1000LL);
    assert(s_startup_retries_left == IMU_STARTUP_RETRY_COUNT);
    assert(!bus_resets && !transport_attaches && !transport_begins);
    assert(!acquire_raw_sample(clock_us, &state));
    assert(agmt_reads == CONFIG_IMU_ERROR_REINIT_COUNT);

    reset_test();
    agmt_result = ICM_20948_STAT_ERR;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(s_sensor_ready);
    agmt_result = ICM_20948_STAT_OK;
    assert(acquire_raw_sample(clock_us, &state));
    assert(s_runtime_recovery_armed && transport_attaches == 1 && !transport_begins);
    assert(acquire_raw_sample(clock_us, &state));
    assert(transport_attaches == 1 && transport_begins == 1);
}

static void test_startup_keeps_probing_after_fast_retries(void) {
    reset_test();
    s_i2c_config.dev_handle = NULL;
    s_sensor_ready = false;
    sample_mode_result = ICM_20948_STAT_ERR;
    shared.ble_connected = true;
    assert(imu_manager_init() == ESP_OK);
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(device_additions == 1 && added_clock_hz == CONFIG_BOARD_I2C_FREQUENCY_HZ);
    assert(sensor_resets == 1 && s_startup_retries_left == IMU_STARTUP_RETRY_COUNT);

    for (unsigned left = IMU_STARTUP_RETRY_COUNT; left > 0; --left) {
        assert(s_recovery_due_us == clock_us + IMU_STARTUP_RETRY_MS * 1000LL);
        clock_us = s_recovery_due_us - 1;
        const unsigned probes = bus_resets;
        retry_sensor_if_due();
        assert(bus_resets == probes);
        ++clock_us;
        retry_sensor_if_due();
        assert(bus_resets == probes + 1 && s_startup_retries_left == left - 1);
        assert(!s_sensor_ready && added_clock_hz == IMU_RECOVERY_CLOCK_HZ);
    }
    assert(s_recovery_due_us == clock_us + IMU_STARTUP_OFFLINE_RETRY_MS * 1000LL);

    clock_us = s_recovery_due_us - 1;
    retry_sensor_if_due();
    assert(bus_resets == IMU_STARTUP_RETRY_COUNT);
    ++clock_us;
    retry_sensor_if_due();
    assert(bus_resets == IMU_STARTUP_RETRY_COUNT + 1 && !s_startup_retries_left);
    assert(s_recovery_due_us == clock_us + IMU_STARTUP_OFFLINE_RETRY_MS * 1000LL);
    assert(!s_sensor_ready && shared.ble_connected && shared.imu_state == IMU_STATE_DEGRADED);

    clock_us = s_recovery_due_us;
    sample_mode_result = ICM_20948_STAT_OK;
    retry_sensor_if_due();
    assert(bus_resets == IMU_STARTUP_RETRY_COUNT + 2 && !s_startup_retries_left);
    assert(s_sensor_ready && added_clock_hz == IMU_RECOVERY_CLOCK_HZ && sensor_resets == 1);
    assert(!s_runtime_recovery_armed && !transport_attaches && !transport_begins);
    const system_state_t state = shared;
    assert(acquire_raw_sample(clock_us, &state));
    assert(s_runtime_recovery_armed && transport_attaches == 1 && !transport_begins);
}

static void test_runtime_recovery_requires_real_acquisition(void) {
    reset_test();
    system_state_t state = {.imu_dmp = true, .ble_connected = true, .requested_imu_rate_hz = 100};
    shared = state;
    assert(acquire_raw_sample(clock_us, &state));
    agmt_transport_error = ESP_ERR_TIMEOUT;
    assert(!acquire_raw_sample(clock_us, &state));
    clock_us = s_recovery_due_us - 1;
    retry_sensor_if_due();
    assert(!bus_resets && !s_sensor_ready);
    ++clock_us;
    retry_sensor_if_due();
    assert(bus_resets == 1 && !device_additions && !device_removals);
    assert(s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(s_recovery_sample_deadline_us == clock_us + IMU_RECOVERY_SAMPLE_TIMEOUT_MS * 1000LL);
    assert(buffered_raw_count == 1);

    /* Identity and power reads succeed, but the subsequent raw SLV4 read NACKs. */
    state.imu_dmp = false;
    agmt_transport_error = ESP_OK;
    raw_aux_nack = true;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(raw_aux_calls == 1 && s_i2c.failed_register == AGB0_REG_I2C_MST_STATUS);
    assert(shared.last_error == ESP_ERR_INVALID_RESPONSE && shared.i2c_errors == 2);
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(s_reconfigure_on_retry);
    assert(s_recovery_due_us == clock_us + 4 * IMU_RECOVERY_FIRST_MS * 1000LL);
    assert(buffered_raw_count == 1);

    clock_us = s_recovery_due_us;
    retry_sensor_if_due();
    assert(bus_resets == 2 && device_removals == 1 && device_additions == 1);
    assert(added_clock_hz == IMU_RECOVERY_CLOCK_HZ && transport_attaches == 2);
    assert(s_i2c.device == s_i2c_config.dev_handle && s_i2c.device == (void *)3);
    assert(s_sensor_ready && s_recovery_sample_deadline_us > clock_us);
    assert(shared.imu_state == IMU_STATE_DEGRADED);

    state.imu_dmp = true;
    raw_aux_nack = false;
    agmt_result = ICM_20948_STAT_ERR;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(s_recovery_due_us == clock_us + 16 * IMU_RECOVERY_FIRST_MS * 1000LL);
    assert(buffered_raw_count == 1);

    clock_us = s_recovery_due_us;
    retry_sensor_if_due();
    assert(bus_resets == 3 && device_additions == 2);
    agmt_result = ICM_20948_STAT_OK;
    assert(acquire_raw_sample(clock_us, &state));
    assert(!s_recovery_sample_deadline_us && s_sensor_ready);
    assert(shared.imu_state == IMU_STATE_RUNNING && buffered_raw_count == 2);
    assert(shared.ble_connected && shared.requested_imu_rate_hz == 100);

    agmt_transport_error = ESP_ERR_TIMEOUT;
    assert(!acquire_raw_sample(clock_us, &state));
    assert(s_recovery_due_us == clock_us + IMU_RECOVERY_FIRST_MS * 1000LL);
}

static void test_runtime_recovery_requires_dmp_frame_before_deadline(void) {
    reset_test();
    confirm_hardware_sample();
    mark_sensor_unavailable(ESP_ERR_TIMEOUT);
    clock_us = s_recovery_due_us;
    retry_sensor_if_due();
    const int64_t deadline = s_recovery_sample_deadline_us;
    assert(deadline > clock_us && s_sensor_ready);
    assert(shared.imu_state == IMU_STATE_DEGRADED);

    unsigned errors = 0;
    imu_dmp_sample_t sample = {0};
    assert(!read_dmp_sample(&sample, &errors)); /* An empty FIFO can be normal. */
    assert(!errors && shared.imu_state == IMU_STATE_DEGRADED);
    clock_us = deadline - 1;
    retry_sensor_if_due();
    assert(s_sensor_ready && bus_resets == 1);
    ++clock_us;
    retry_sensor_if_due();
    assert(!s_sensor_ready && shared.imu_state == IMU_STATE_DEGRADED);
    assert(shared.last_error == ESP_ERR_TIMEOUT && s_reconfigure_on_retry);
    assert(s_recovery_due_us == clock_us + 4 * IMU_RECOVERY_FIRST_MS * 1000LL);
}

int main(void) {
    test_independent_outputs();
    test_preserved_fifo_batch();
    test_agmt_failure();
    test_repeated_raw_failures_stop_acquisition();
    test_timeout_schedules_fast_recovery();
    test_recovery_only_arms_after_first_sample();
    test_startup_keeps_probing_after_fast_retries();
    test_runtime_recovery_requires_real_acquisition();
    test_runtime_recovery_requires_dmp_frame_before_deadline();
    test_latest_and_abort();
    puts("PASS: Compass-only, quaternion-only, calibrated isolation, native counts, raw DMP publication, freshness boundary");
    puts("PASS: latest frame, incomplete rollback, frame/time backlog caps, reset failure, explicit reset invalidation (ASan/UBSan)");
    test_terminal_deadline();
    test_transport_error_recovery();
    test_reset_gate();
    puts("PASS: unconditional terminal deadline (4999/5000/20000us), destructive error rollback, failed-reset read gate, recovery, manual reset gate");
    puts("PASS: bank cache invalidated before every reset attempt, including retries after cache-poisoning failures");
    puts("PASS: AGMT failure in raw/DMP modes invalidates magnetic and bank caches, increments dropped count, preserves published sample and sequence");
    puts("PASS: first runtime timeout schedules recovery; bus reset preserves handle; sleeping IMU requires reinitialization");
    puts("PASS: repeated failures before the first sample schedule a startup retry, and the first valid sample uses the stock transport");
    puts("PASS: startup continues slow stock-driver probes at 100 kHz after three fast retries");
    puts("PASS: runtime recovery stays degraded until a hardware sample, escalates on raw SLV4 NACK or empty DMP FIFO deadline");
    return 0;
}
