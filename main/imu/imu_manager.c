#include "imu_manager.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>
#include "magnetometer_reader.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "board/board_config.h"
#include "system_state.h"
#include "ble/ble_sample_buffer.h"
#include "icm20948.h"
#include "icm20948_i2c.h"
#include "imu_i2c_transport.h"

static const char *TAG = "imu";
#if CONFIG_IMU_ENABLED
static icm20948_device_t s_sensor;
static imu_magnetometer_t s_magnetometer;
static icm0948_config_i2c_t s_i2c_config;
static imu_i2c_transport_t s_i2c;
enum {
    IMU_STREAM_TIMEOUT_MS = 20,
    IMU_STARTUP_RETRY_MS = 1000,
    IMU_STARTUP_OFFLINE_RETRY_MS = 10000,
    IMU_STARTUP_RETRY_COUNT = 3,
    IMU_RECOVERY_CLOCK_HZ = 100000,
    IMU_RECOVERY_FIRST_MS = 50,
    IMU_RECOVERY_MAX_MS = 5000,
    IMU_RECOVERY_SAMPLE_TIMEOUT_MS = 2000,
};
static unsigned s_raw_read_failures;
static unsigned s_dmp_reset_failures;
static int64_t s_recovery_due_us;
/* Only the acquisition task writes this deadline. Other tasks read it under
 * s_device_lock when reporting a transport failure. */
static int64_t s_recovery_sample_deadline_us;
static unsigned s_recovery_delay_ms;
static unsigned s_startup_retries_left;
static uint32_t s_i2c_clock_hz = CONFIG_BOARD_I2C_FREQUENCY_HZ;
static bool s_reconfigure_on_retry;
static bool s_runtime_recovery_armed;
#endif
static SemaphoreHandle_t s_device_lock;
static SemaphoreHandle_t s_dmp_sample_lock;
static SemaphoreHandle_t s_raw_sample_lock;
static imu_dmp_sample_t s_latest_dmp_sample;
static imu_raw_sample_t s_latest_raw_sample;
static TaskHandle_t s_acquisition_task;
static atomic_bool s_acquisition_requested;
static atomic_bool s_acquisition_active;
static atomic_bool s_sensor_ready;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
static uint32_t s_dmp_sequence;
#if CONFIG_IMU_DMP_ENABLED
static bool s_dmp_fifo_needs_reset;
static imu_dmp_sample_t s_dmp_accepted[8];
static size_t s_dmp_accepted_count;
#endif
#endif
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
static uint32_t s_raw_sequence;
#endif

#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
static int32_t quaternion_to_q30(double value)
{
    if (value > 1.0) value = 1.0;
    if (value < -1.0) value = -1.0;
    return (int32_t)llround(value * 1073741824.0);
}

static void publish_dmp_sample(const imu_dmp_sample_t *sample, uint32_t epoch)
{
    xSemaphoreTake(s_dmp_sample_lock, portMAX_DELAY);
    s_latest_dmp_sample = *sample;
    xSemaphoreGive(s_dmp_sample_lock);
    ble_sample_buffer_publish_dmp(sample, epoch);
    SYSTEM_STATE_INCREMENT(dmp_sample_count);
}
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
static void publish_raw_sample(const imu_raw_sample_t *sample, uint32_t epoch)
{
    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    s_latest_raw_sample = *sample;
    xSemaphoreGive(s_raw_sample_lock);
    ble_sample_buffer_publish_raw(sample, epoch);
    SYSTEM_STATE_INCREMENT(raw_sample_count);
}
#endif

#if CONFIG_IMU_DMP_DUMMY || (CONFIG_IMU_SYNTHETIC_FALLBACK && CONFIG_IMU_DMP_ENABLED)
static void generate_dmp_sample(imu_dmp_sample_t *sample)
{
    const double angle = (double)(s_dmp_sequence % 1000) *
                         (2.0 * M_PI / 1000.0);
    sample->q0_q30 = quaternion_to_q30(cos(angle * 0.5));
    sample->q1_q30 = 0;
    sample->q2_q30 = 0;
    sample->q3_q30 = quaternion_to_q30(sin(angle * 0.5));
    sample->flags = IMU_FLAG_VALID | IMU_FLAG_SYNTHETIC;
}
#endif

/* Evaluate each driver call once and retain its native status in boot logs. */
#define IMU_INIT_CHECK(call, error) do { \
    icm20948_status_e init_rc_ = (call); \
    if (init_rc_ != ICM_20948_STAT_OK) { \
        ESP_LOGE(TAG, "IMU init: %s failed: rc=%d (0x%x), returning %s", \
                 #call, (int)init_rc_, (unsigned)init_rc_, esp_err_to_name(error)); \
        return (error); \
    } \
} while (0)

#if CONFIG_IMU_ENABLED
static esp_err_t configure_sensor(bool cold_start, bool reset_sensor)
{
    memset(&s_magnetometer, 0, sizeof(s_magnetometer));
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_IMU_I2C_ADDRESS,
        .scl_speed_hz = s_i2c_clock_hz,
    };
    esp_err_t error = i2c_master_bus_add_device(board_i2c_bus(), &device_config,
                                               &s_i2c_config.dev_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "IMU init: i2c_master_bus_add_device failed: %s (0x%x), address=0x%02x",
                 esp_err_to_name(error), (unsigned)error, CONFIG_IMU_I2C_ADDRESS);
        return error;
    }

    /* The driver initializes its bank cache, DMP state and bus interface. */
    icm20948_init_i2c(&s_sensor, &s_i2c_config);

    uint8_t identity = 0;
    icm20948_status_e status = ICM_20948_STAT_ERR;
    for (unsigned attempt = 0; attempt < CONFIG_IMU_I2C_RETRIES; ++attempt) {
        if (!cold_start) s_sensor._last_bank = UINT8_MAX;
        status = icm20948_get_who_am_i(&s_sensor, &identity);
        ESP_LOGI(TAG, "WHO_AM_I attempt %u: rc=%d identity=0x%02x",
                 attempt + 1, status, identity);
        if (status == ICM_20948_STAT_OK && identity == ICM_20948_WHOAMI) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (status != ICM_20948_STAT_OK || identity != ICM_20948_WHOAMI) {
        ESP_LOGE(TAG, "IMU init: icm20948_get_who_am_i failed after %u attempts: "
                 "rc=%d (0x%x), identity=0x%02x expected=0x%02x",
                 (unsigned)CONFIG_IMU_I2C_RETRIES, (int)status, (unsigned)status,
                 identity, ICM_20948_WHOAMI);
        return ESP_ERR_NOT_FOUND;
    }
    SYSTEM_STATE_SET(who_am_i, identity);

    if (reset_sensor) {
        vTaskDelay(pdMS_TO_TICKS(50));
        status = icm20948_sw_reset(&s_sensor);
        if (status != ICM_20948_STAT_OK) {
            ESP_LOGE(TAG, "IMU init: icm20948_sw_reset failed: rc=%d (0x%x); continuing",
                     (int)status, (unsigned)status);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        s_sensor._last_bank = 255; /* Reset invalidates the driver's bank cache. */
    }
    IMU_INIT_CHECK(icm20948_sleep(&s_sensor, false), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_low_power(&s_sensor, false), ESP_FAIL);
    vTaskDelay(pdMS_TO_TICKS(50));

    const icm20948_internal_sensor_id_bm sensors =
        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    IMU_INIT_CHECK(icm20948_set_sample_mode(&s_sensor, sensors, SAMPLE_MODE_CONTINUOUS), ESP_FAIL);
    const icm20948_fss_t full_scale = {.a = GPM_2, .g = DPS_250};
    IMU_INIT_CHECK(icm20948_set_full_scale(&s_sensor, sensors, full_scale), ESP_FAIL);
    const icm20948_dlpcfg_t low_pass_filter = {
        .a = ACC_D473BW_N499BW,
        .g = GYR_D361BW4_N376BW5,
    };
    IMU_INIT_CHECK(icm20948_set_dlpf_cfg(&s_sensor, sensors, low_pass_filter), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_dlpf(&s_sensor, ICM_20948_INTERNAL_ACC, false), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_dlpf(&s_sensor, ICM_20948_INTERNAL_GYR, false), ESP_FAIL);
#if CONFIG_IMU_DMP_ENABLED
    /* Leave the AK09916 powered down for the DMP's single-shot controller. */
    const bool dmp_owns_magnetometer = true;
#else
    const bool dmp_owns_magnetometer = false;
#endif
    IMU_INIT_CHECK(imu_magnetometer_configure(&s_sensor, &s_magnetometer, dmp_owns_magnetometer),
                   ESP_ERR_NOT_FOUND);

#if CONFIG_IMU_DMP_ENABLED
    IMU_INIT_CHECK(icm20948_init_dmp_sensor_with_defaults(&s_sensor), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_enable_dmp_sensor(
        &s_sensor, INV_ICM20948_SENSOR_ORIENTATION, 1), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_set_dmp_sensor_period(
        &s_sensor, DMP_ODR_Reg_Quat9, 0), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_enable_dmp_sensor(
        &s_sensor, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_set_dmp_sensor_period(
        &s_sensor, DMP_ODR_Reg_Cpass, 0), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_fifo(&s_sensor, true), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_dmp(&s_sensor, true), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_reset_dmp(&s_sensor), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_reset_fifo(&s_sensor), ESP_FAIL);
    s_dmp_fifo_needs_reset = false;
    SYSTEM_STATE_SET(imu_dmp, true);
    if (cold_start) SYSTEM_STATE_SET(requested_imu_rate_hz, CONFIG_IMU_DMP_RATE_HZ);
    /* These are estimates until the acquisition task measures actual output. */
#if defined(CONFIG_IMU_DMP_MANAGED_RATE_HZ)
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, CONFIG_IMU_DMP_MANAGED_RATE_HZ);
#elif defined(CONFIG_IMU_DMP_OVERRIDE_RATE_HZ)
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, CONFIG_IMU_DMP_OVERRIDE_RATE_HZ);
#else
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, 0);
#endif
#endif
    return ESP_OK;
}

static esp_err_t detach_sensor(void)
{
    if (!s_i2c_config.dev_handle) return ESP_OK;
    const esp_err_t error = i2c_master_bus_rm_device(s_i2c_config.dev_handle);
    if (error == ESP_OK) {
        s_i2c_config.dev_handle = NULL;
        s_i2c.device = NULL;
    }
    return error;
}

static void schedule_startup_retry(esp_err_t error)
{
    atomic_store(&s_sensor_ready, false);
    SYSTEM_STATE_SET(who_am_i, 0);
    SYSTEM_STATE_SET(imu_dmp, false);
    SYSTEM_STATE_SET(imu_state, IMU_STATE_DEGRADED);
    system_state_set_error("imu", error);
    if (s_startup_retries_left) {
        s_recovery_due_us = esp_timer_get_time() + IMU_STARTUP_RETRY_MS * 1000LL;
        ESP_LOGW(TAG, "IMU unavailable before first sample: %s; %u startup retries left",
                 esp_err_to_name(error), s_startup_retries_left);
    } else {
        s_recovery_due_us = esp_timer_get_time() + IMU_STARTUP_OFFLINE_RETRY_MS * 1000LL;
        ESP_LOGW(TAG, "IMU still offline before first sample: %s; retrying in %u ms",
                 esp_err_to_name(error), IMU_STARTUP_OFFLINE_RETRY_MS);
    }
}

/* Call with the device lock after acquiring a valid hardware sample. */
static void confirm_hardware_sample(void)
{
    if (!s_runtime_recovery_armed) {
        s_i2c.device = s_i2c_config.dev_handle;
        imu_i2c_attach(&s_i2c, &s_sensor);
        s_runtime_recovery_armed = true;
        ESP_LOGI(TAG, "IMU streaming; runtime I2C recovery armed");
    } else if (s_recovery_sample_deadline_us) {
        s_recovery_sample_deadline_us = 0;
        SYSTEM_STATE_SET(imu_state, IMU_STATE_RUNNING);
        ESP_LOGI(TAG, "IMU recovered at %lu Hz I2C; hardware samples resumed",
                 (unsigned long)s_i2c_clock_hz);
    }
}

static void log_i2c_failure(const char *operation, esp_err_t error)
{
    int scl, sda;
    board_i2c_line_levels(&scl, &sda);
    ESP_LOGW(TAG, "%s: %s (%s reg=0x%02x), SCL=%d SDA=%d",
             operation, esp_err_to_name(error), s_i2c.failed_write ? "write" : "read",
             (unsigned)s_i2c.failed_register, scl, sda);
}

static void schedule_runtime_retry(esp_err_t error)
{
    system_state_set_error("imu", error);
    s_recovery_delay_ms *= 4;
    if (s_recovery_delay_ms > IMU_RECOVERY_MAX_MS) s_recovery_delay_ms = IMU_RECOVERY_MAX_MS;
    s_recovery_due_us = esp_timer_get_time() + s_recovery_delay_ms * 1000LL;
    ESP_LOGW(TAG, "IMU recovery failed: %s; retrying in %u ms",
             esp_err_to_name(error), s_recovery_delay_ms);
}

/* Call under the device lock. */
static void mark_sensor_unavailable(esp_err_t error)
{
    if (!s_runtime_recovery_armed) {
        schedule_startup_retry(error);
        return;
    }
    if (s_recovery_sample_deadline_us) {
        s_reconfigure_on_retry = true;
        schedule_runtime_retry(error);
    } else {
        s_reconfigure_on_retry = false;
        s_recovery_delay_ms = IMU_RECOVERY_FIRST_MS;
        s_recovery_due_us = esp_timer_get_time() + s_recovery_delay_ms * 1000LL;
        system_state_set_error("imu", error);
    }
    atomic_store(&s_sensor_ready, false);
    ble_sample_buffer_invalidate(BLE_BUFFER_RAW);
    ble_sample_buffer_invalidate(BLE_BUFFER_DMP);
#if !CONFIG_IMU_RAW_DUMMY
    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    s_latest_raw_sample.flags = 0;
    xSemaphoreGive(s_raw_sample_lock);
    SYSTEM_STATE_SET(actual_raw_sample_rate_hz, 0);
#endif
#if !CONFIG_IMU_DMP_DUMMY
    xSemaphoreTake(s_dmp_sample_lock, portMAX_DELAY);
    s_latest_dmp_sample.flags = 0;
    xSemaphoreGive(s_dmp_sample_lock);
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, 0);
#endif
    SYSTEM_STATE_SET(who_am_i, 0);
    SYSTEM_STATE_SET(imu_dmp, false);
    SYSTEM_STATE_SET(imu_state, IMU_STATE_DEGRADED);
}

static esp_err_t reset_i2c_bus(void)
{
    esp_err_t error = i2c_master_bus_reset(board_i2c_bus());
    if (error != ESP_OK) return error;
    int scl, sda;
    board_i2c_line_levels(&scl, &sda);
    if (!scl || !sda) {
        ESP_LOGW(TAG, "I2C bus still held low after reset: SCL=%d SDA=%d", scl, sda);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* A bus clear alone often leaves the sensor configuration intact. A full
 * reinitialization is reserved for later attempts if the in-place check fails. */
static esp_err_t verify_runtime_sensor(void)
{
    s_sensor._last_bank = 255;
    s_sensor._last_mems_bank = 255;
    imu_i2c_begin(&s_i2c, IMU_STREAM_TIMEOUT_MS);
    uint8_t identity = 0;
    icm20948_status_e status = icm20948_get_who_am_i(&s_sensor, &identity);
    if (s_i2c.first_error != ESP_OK) return s_i2c.first_error;
    if (status != ICM_20948_STAT_OK || identity != ICM_20948_WHOAMI) return ESP_ERR_NOT_FOUND;
    icm20948_pwr_mgmt_1_t power;
    status = icm20948_execute_r(&s_sensor, AGB0_REG_PWR_MGMT_1, (uint8_t *)&power, sizeof(power));
    if (s_i2c.first_error != ESP_OK) return s_i2c.first_error;
    if (status != ICM_20948_STAT_OK || power.SLEEP || power.DEVICE_RESET) return ESP_ERR_INVALID_STATE;
#if CONFIG_IMU_DMP_ENABLED
    status = icm20948_reset_fifo(&s_sensor);
    if (s_i2c.first_error != ESP_OK) return s_i2c.first_error;
    if (status != ICM_20948_STAT_OK) return ESP_FAIL;
    s_dmp_fifo_needs_reset = false;
    SYSTEM_STATE_SET(imu_dmp, true);
#endif
    imu_magnetometer_invalidate(&s_magnetometer);
    SYSTEM_STATE_SET(who_am_i, identity);
    return ESP_OK;
}

static void retry_sensor_if_due(void)
{
    if (atomic_load(&s_sensor_ready)) {
        if (s_recovery_sample_deadline_us &&
            esp_timer_get_time() >= s_recovery_sample_deadline_us) {
            xSemaphoreTake(s_device_lock, portMAX_DELAY);
            if (atomic_load(&s_sensor_ready)) {
                ESP_LOGW(TAG, "IMU recovery produced no hardware sample");
                mark_sensor_unavailable(ESP_ERR_TIMEOUT);
            }
            xSemaphoreGive(s_device_lock);
        }
        return;
    }
    xSemaphoreTake(s_device_lock, portMAX_DELAY);
    if (!s_recovery_due_us || esp_timer_get_time() < s_recovery_due_us) {
        xSemaphoreGive(s_device_lock);
        return;
    }
    s_recovery_sample_deadline_us = 0;
    SYSTEM_STATE_INCREMENT(imu_reinitializations);
    const bool runtime_recovery = s_runtime_recovery_armed;
    if (!runtime_recovery) {
        if (s_startup_retries_left) --s_startup_retries_left;
        if (s_i2c_clock_hz > IMU_RECOVERY_CLOCK_HZ)
            s_i2c_clock_hz = IMU_RECOVERY_CLOCK_HZ;
        ESP_LOGI(TAG, "Retrying IMU startup with stock driver at %lu Hz",
                 (unsigned long)s_i2c_clock_hz);
    }

    esp_err_t error = reset_i2c_bus();
    if (error == ESP_OK) {
        if (!runtime_recovery || s_reconfigure_on_retry) {
            error = detach_sensor();
            if (error == ESP_OK && runtime_recovery && s_i2c_clock_hz > IMU_RECOVERY_CLOCK_HZ) {
                s_i2c_clock_hz = IMU_RECOVERY_CLOCK_HZ;
                ESP_LOGW(TAG, "Retrying IMU at %lu Hz I2C", (unsigned long)s_i2c_clock_hz);
            }
            if (error == ESP_OK) error = configure_sensor(false, runtime_recovery);
            if (error == ESP_OK && runtime_recovery) {
                s_i2c.device = s_i2c_config.dev_handle;
                imu_i2c_attach(&s_i2c, &s_sensor);
            }
        } else {
            error = verify_runtime_sensor();
            if (s_i2c.first_error != ESP_OK) log_i2c_failure("IMU recovery check failed", error);
            s_reconfigure_on_retry = error != ESP_OK;
        }
    }
    if (error == ESP_OK) {
        s_raw_read_failures = 0;
        s_dmp_reset_failures = 0;
        atomic_store(&s_sensor_ready, true);
        if (runtime_recovery) {
            s_recovery_sample_deadline_us =
                esp_timer_get_time() + IMU_RECOVERY_SAMPLE_TIMEOUT_MS * 1000LL;
            ESP_LOGI(TAG, "IMU responds at %lu Hz I2C; waiting for a hardware sample",
                     (unsigned long)s_i2c_clock_hz);
        } else {
            SYSTEM_STATE_SET(imu_state, IMU_STATE_RUNNING);
            ESP_LOGI(TAG, "IMU configured after startup retry; starting acquisition");
        }
    } else if (!s_runtime_recovery_armed) {
        schedule_startup_retry(error);
    } else {
        schedule_runtime_retry(error);
    }
    xSemaphoreGive(s_device_lock);
}

/* Must run before any new transport operation can clear the first error. */
static void record_i2c_failure(const char *operation, unsigned *consecutive_failures)
{
    const esp_err_t error = s_i2c.first_error;
    if (error == ESP_OK) return;
    SYSTEM_STATE_INCREMENT(i2c_errors);
    const bool first_failure = ++*consecutive_failures == 1;
    const bool needs_recovery = s_recovery_sample_deadline_us || error == ESP_ERR_TIMEOUT ||
        *consecutive_failures >= CONFIG_IMU_ERROR_REINIT_COUNT;
    if (first_failure || needs_recovery) log_i2c_failure(operation, error);
    if (needs_recovery) mark_sensor_unavailable(error);
}
#endif

#if CONFIG_IMU_DMP_ENABLED
/* All FIFO and cache accesses below require s_device_lock. */
static bool recover_dmp_fifo(void)
{
    ble_sample_buffer_invalidate(BLE_BUFFER_DMP);
    imu_magnetometer_invalidate(&s_magnetometer);
    /* The driver caches bank changes before confirming the host write. */
    s_sensor._last_bank = 255;
    if (s_runtime_recovery_armed) {
        s_sensor._last_mems_bank = 255;
        imu_i2c_begin(&s_i2c, IMU_STREAM_TIMEOUT_MS);
    }
    const icm20948_status_e status = icm20948_reset_fifo(&s_sensor);
    s_dmp_fifo_needs_reset = status != ICM_20948_STAT_OK || s_i2c.first_error != ESP_OK;
    SYSTEM_STATE_INCREMENT(imu_reinitializations);
    if (s_i2c.first_error != ESP_OK) {
        record_i2c_failure("DMP FIFO reset I2C failure", &s_dmp_reset_failures);
    } else if (s_dmp_fifo_needs_reset) {
        system_state_set_error("imu_fifo", ESP_FAIL);
        if (!s_runtime_recovery_armed && ++s_dmp_reset_failures >= CONFIG_IMU_ERROR_REINIT_COUNT)
            mark_sensor_unavailable(ESP_FAIL);
    } else {
        s_dmp_reset_failures = 0;
    }
    return !s_dmp_fifo_needs_reset;
}

static void decode_dmp_frame(const icm_20948_DMP_data_t *fifo_data,
                             imu_dmp_sample_t *sample, uint32_t timestamp_ms)
{
    double q1 = (double)fifo_data->Quat9.Data.Q1 / 1073741824.0;
    double q2 = (double)fifo_data->Quat9.Data.Q2 / 1073741824.0;
    double q3 = (double)fifo_data->Quat9.Data.Q3 / 1073741824.0;
    double q0 = sqrt(fmax(0.0, 1.0 - q1*q1 - q2*q2 - q3*q3));
    const double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    *sample = (imu_dmp_sample_t){
        .sequence = 0,
        .timestamp_ms = timestamp_ms,
        .q0_q30 = quaternion_to_q30(q0),
        .q1_q30 = quaternion_to_q30(q1),
        .q2_q30 = quaternion_to_q30(q2),
        .q3_q30 = quaternion_to_q30(q3),
        .accuracy = (uint16_t)fifo_data->Quat9.Data.Accuracy,
        .flags = IMU_FLAG_VALID | IMU_FLAG_DMP,
    };
}

static bool read_dmp_sample(imu_dmp_sample_t *sample, unsigned *consecutive_errors)
{
    s_dmp_accepted_count = 0;
    if (s_dmp_fifo_needs_reset && !recover_dmp_fifo()) return false;
    icm_20948_DMP_data_t fifo_data = {0};
    imu_magnetometer_t pending_mag = s_magnetometer;
    size_t accepted_count = 0;
    bool compass_ready = false;
    const int64_t started_us = esp_timer_get_time();
    icm20948_status_e status;
    unsigned frames = 0;
    if (s_runtime_recovery_armed) imu_i2c_begin(&s_i2c, IMU_STREAM_TIMEOUT_MS);
    do {
        status = inv_icm20948_read_dmp_data(&s_sensor, &fifo_data);
        if (s_i2c.first_error != ESP_OK) {
            SYSTEM_STATE_INCREMENT(dropped_dmp_samples);
            imu_magnetometer_invalidate(&s_magnetometer);
            s_dmp_fifo_needs_reset = true;
            record_i2c_failure("DMP FIFO read I2C failure", consecutive_errors);
            return false;
        }
        if (status == ICM_20948_STAT_FIFO_NO_DATA_AVAIL) break;
        if (status != ICM_20948_STAT_OK && status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            imu_magnetometer_invalidate(&s_magnetometer);
            SYSTEM_STATE_INCREMENT(dropped_dmp_samples);
            /* Any transport error may follow a destructive partial read.
             * Resume only after a successful reset restores frame alignment. */
            if (++*consecutive_errors >= CONFIG_IMU_ERROR_REINIT_COUNT) {
                if (status == ICM_20948_STAT_ERR) {
                    SYSTEM_STATE_INCREMENT(i2c_errors);
                    ESP_LOGW(TAG, "Repeated DMP read failures; scheduling IMU recovery");
                    mark_sensor_unavailable(ESP_FAIL);
                    *consecutive_errors = 0;
                    return false;
                }
                SYSTEM_STATE_SET(imu_state, IMU_STATE_DEGRADED);
            }
            s_dmp_fifo_needs_reset = true;
            (void)recover_dmp_fifo();
            return false;
        }
        if (fifo_data.header & DMP_header_bitmap_Compass) {
            /* FIFO Compass is signed AK counts, unlike calibrated uT-Q16. */
            imu_magnetometer_update(&pending_mag, fifo_data.Compass.Data.X,
                fifo_data.Compass.Data.Y, fifo_data.Compass.Data.Z, started_us);
            compass_ready = true;
        }
        if (fifo_data.header & DMP_header_bitmap_Quat9) {
            decode_dmp_frame(&fifo_data, &s_dmp_accepted[accepted_count++],
                             (uint32_t)(esp_timer_get_time() / 1000ULL));
        }
        ++frames;
    } while (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL && frames < 8 &&
             esp_timer_get_time() - started_us < 5000);
    *consecutive_errors = 0;

    if (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL ||
        esp_timer_get_time() - started_us >= 5000) {
        /* Discard an undrained backlog instead of giving old compass data a
         * fresh dequeue timestamp. Bound FIFO work so raw/BLE tasks can run. */
        s_dmp_fifo_needs_reset = true;
        (void)recover_dmp_fifo();
        SYSTEM_STATE_INCREMENT(dropped_dmp_samples);
        return false;
    }
    if (compass_ready) s_magnetometer = pending_mag;
    if (!accepted_count) return false;

    for (size_t i = 0; i < accepted_count; ++i)
        s_dmp_accepted[i].sequence = ++s_dmp_sequence;
    s_dmp_accepted_count = accepted_count;
    *sample = s_dmp_accepted[accepted_count - 1];
    return true;
}
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
static bool acquire_raw_sample(uint64_t now_us, const system_state_t *state)
{
    uint32_t epoch;
    imu_raw_sample_t sample = {
        .timestamp_ms = (uint32_t)(now_us / 1000ULL),
        .actual_rate_hz = (uint16_t)state->actual_raw_sample_rate_hz,
        .flags = IMU_RAW_FLAG_VALID | IMU_RAW_FLAG_ACCEL_VALID |
                 IMU_RAW_FLAG_GYRO_VALID | IMU_RAW_FLAG_MAG_VALID | IMU_RAW_FLAG_TEMP_VALID,
    };
#if CONFIG_IMU_RAW_DUMMY
    epoch = ble_sample_buffer_epoch(BLE_BUFFER_RAW);
    sample.sequence = ++s_raw_sequence;
    const int16_t generated_value = (int16_t)((int32_t)(s_raw_sequence % 65536) - 32768);
    sample.accel_x = sample.accel_y = sample.accel_z = generated_value;
    sample.gyro_x = sample.gyro_y = sample.gyro_z = generated_value;
    sample.mag_x = sample.mag_y = sample.mag_z = generated_value;
    sample.temperature = generated_value;
#else
    if (!s_sensor_ready) return false;
    icm20948_agmt_t sensor_data = {0};
    xSemaphoreTake(s_device_lock, portMAX_DELAY);
    if (!s_sensor_ready) {
        xSemaphoreGive(s_device_lock);
        return false;
    }
    epoch = ble_sample_buffer_epoch(BLE_BUFFER_RAW);
    if (s_runtime_recovery_armed) imu_i2c_begin(&s_i2c, IMU_STREAM_TIMEOUT_MS);
    icm20948_status_e status = icm20948_get_agmt(&s_sensor, &sensor_data);
    if (status == ICM_20948_STAT_OK && s_i2c.first_error == ESP_OK && !state->imu_dmp &&
        imu_magnetometer_poll(&s_sensor, &s_magnetometer, now_us) != ICM_20948_STAT_OK &&
        s_i2c.first_error == ESP_OK) {
        system_state_set_error("imu_mag", ESP_FAIL);
    }
    if (status != ICM_20948_STAT_OK || s_i2c.first_error != ESP_OK) {
        s_sensor._last_bank = 255;
        if (s_runtime_recovery_armed) s_sensor._last_mems_bank = 255;
        imu_magnetometer_invalidate(&s_magnetometer);
        SYSTEM_STATE_INCREMENT(dropped_raw_samples);
        if (s_i2c.first_error != ESP_OK) {
            record_i2c_failure("Raw IMU acquisition I2C failure", &s_raw_read_failures);
        } else {
            if (++s_raw_read_failures == 1) ESP_LOGW(TAG, "Raw IMU read failed: rc=%d", (int)status);
            if (s_recovery_sample_deadline_us || s_raw_read_failures >= CONFIG_IMU_ERROR_REINIT_COUNT)
                mark_sensor_unavailable(ESP_FAIL);
        }
        xSemaphoreGive(s_device_lock);
        return false;
    }
    s_raw_read_failures = 0;
    imu_magnetometer_apply(&s_magnetometer, &sample, esp_timer_get_time());
    confirm_hardware_sample();
    xSemaphoreGive(s_device_lock);
    sample.sequence = ++s_raw_sequence;
    sample.accel_x = sensor_data.acc.axes.x;
    sample.accel_y = sensor_data.acc.axes.y;
    sample.accel_z = sensor_data.acc.axes.z;
    sample.gyro_x = sensor_data.gyr.axes.x;
    sample.gyro_y = sensor_data.gyr.axes.y;
    sample.gyro_z = sensor_data.gyr.axes.z;
    sample.temperature = sensor_data.tmp.val;
    sample.accel_range_code = sensor_data.fss.a;
    sample.gyro_range_code = sensor_data.fss.g;
#endif
    if (state->imu_dmp) sample.flags |= IMU_RAW_FLAG_DMP_CONCURRENT;
    publish_raw_sample(&sample, epoch);
    return true;
}
#endif

static void acquisition_task(void *arg)
{
    (void)arg;
    TickType_t next_wake_tick = xTaskGetTickCount();
    uint64_t measurement_start_us = esp_timer_get_time();
    uint32_t dmp_sample_count = 0;
    uint32_t raw_sample_count = 0;
#if CONFIG_IMU_DMP_ENABLED
    unsigned consecutive_errors = 0;
#endif
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY || CONFIG_IMU_DMP_DUMMY || (CONFIG_IMU_SYNTHETIC_FALLBACK && CONFIG_IMU_DMP_ENABLED)
    uint64_t next_sample_us = 0;
    uint32_t last_sample_rate_hz = 0;
#endif

    while (atomic_load(&s_acquisition_requested)) {
#if CONFIG_IMU_ENABLED
        retry_sensor_if_due();
#endif
        system_state_t state;
        system_state_snapshot(&state);
        const uint32_t requested_imu_rate_hz = state.requested_imu_rate_hz ? state.requested_imu_rate_hz : 1;
        uint32_t polling_rate_hz = requested_imu_rate_hz;
#if CONFIG_IMU_DMP_ENABLED && defined(CONFIG_IMU_FIFO_POLL_RATE_HZ)
        if (polling_rate_hz < CONFIG_IMU_FIFO_POLL_RATE_HZ)
            polling_rate_hz = CONFIG_IMU_FIFO_POLL_RATE_HZ;
#endif
        TickType_t polling_period = pdMS_TO_TICKS(1000 / polling_rate_hz);
        if (!polling_period) polling_period = 1;
        const uint64_t now_us = esp_timer_get_time();
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY || CONFIG_IMU_DMP_DUMMY || (CONFIG_IMU_SYNTHETIC_FALLBACK && CONFIG_IMU_DMP_ENABLED)
        if (requested_imu_rate_hz != last_sample_rate_hz) {
            next_sample_us = now_us;
            last_sample_rate_hz = requested_imu_rate_hz;
        }
        const bool sample_due = now_us >= next_sample_us;
#endif

#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
        imu_dmp_sample_t sample = {.timestamp_ms = (uint32_t)(now_us / 1000ULL)};
        bool valid = false;
        uint32_t dmp_epoch = ble_sample_buffer_epoch(BLE_BUFFER_DMP);
#if CONFIG_IMU_DMP_ENABLED
        if (s_sensor_ready) {
            xSemaphoreTake(s_device_lock, portMAX_DELAY);
            if (s_sensor_ready) {
                dmp_epoch = ble_sample_buffer_epoch(BLE_BUFFER_DMP);
                valid = read_dmp_sample(&sample, &consecutive_errors);
                if (valid) confirm_hardware_sample();
            }
            xSemaphoreGive(s_device_lock);
        }
#endif
#if CONFIG_IMU_DMP_DUMMY || (CONFIG_IMU_SYNTHETIC_FALLBACK && CONFIG_IMU_DMP_ENABLED)
#if CONFIG_IMU_DMP_DUMMY
        const bool generate_sample = true;
#else
        const bool generate_sample = !s_sensor_ready;
#endif
        if (generate_sample && sample_due) {
            sample.sequence = ++s_dmp_sequence;
            generate_dmp_sample(&sample);
            valid = true;
        }
#endif
        if (valid) {
#if CONFIG_IMU_DMP_ENABLED
            if (s_sensor_ready) {
                /* Commit every accepted FIFO quaternion after releasing the
                 * device lock; failed/partial drains publish nothing. */
                for (size_t i = 0; i < s_dmp_accepted_count; ++i)
                    publish_dmp_sample(&s_dmp_accepted[i], dmp_epoch);
                dmp_sample_count += s_dmp_accepted_count;
            } else
#endif
            {
                publish_dmp_sample(&sample, dmp_epoch);
                ++dmp_sample_count;
            }
        }
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY || CONFIG_IMU_DMP_DUMMY || (CONFIG_IMU_SYNTHETIC_FALLBACK && CONFIG_IMU_DMP_ENABLED)
        if (sample_due) {
            /* Keep deadline phase at rates that do not divide the RTOS tick,
             * and skip missed periods without a burst of catch-up I2C reads. */
            const uint64_t sample_period_us = 1000000ULL / requested_imu_rate_hz;
            next_sample_us += ((now_us - next_sample_us) / sample_period_us + 1) *
                              sample_period_us;
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
            if (acquire_raw_sample(now_us, &state)) ++raw_sample_count;
#endif
        }
#endif
        const uint64_t elapsed_us = now_us - measurement_start_us;
        if (elapsed_us >= 1000000ULL) {
            bool sensor_ready = atomic_load(&s_sensor_ready);
#if CONFIG_IMU_ENABLED
            sensor_ready = sensor_ready && !s_recovery_sample_deadline_us;
#endif
            uint32_t measured_dmp_rate_hz = dmp_sample_count * 1000000ULL / elapsed_us;
#if !CONFIG_IMU_DMP_DUMMY && !CONFIG_IMU_SYNTHETIC_FALLBACK
            if (!sensor_ready) measured_dmp_rate_hz = 0;
#endif
            SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, measured_dmp_rate_hz);
            uint32_t measured_raw_rate_hz = raw_sample_count * 1000000ULL / elapsed_us;
#if !CONFIG_IMU_RAW_DUMMY
            if (!sensor_ready) measured_raw_rate_hz = 0;
#endif
            SYSTEM_STATE_SET(actual_raw_sample_rate_hz, measured_raw_rate_hz);
            SYSTEM_STATE_SET(imu_state, s_sensor_ready ? IMU_STATE_RUNNING : IMU_STATE_DEGRADED);
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
            xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
            s_latest_raw_sample.actual_rate_hz = (uint16_t)measured_raw_rate_hz;
            xSemaphoreGive(s_raw_sample_lock);
#endif
            dmp_sample_count = raw_sample_count = 0;
            measurement_start_us = now_us;
        }
#if CONFIG_IMU_ENABLED
        if (!atomic_load(&s_sensor_ready)) {
            const TickType_t recovery_poll = pdMS_TO_TICKS(IMU_RECOVERY_FIRST_MS);
            if (recovery_poll && recovery_poll < polling_period) polling_period = recovery_poll;
        }
        /* A slow transaction or reinitialization must not cause catch-up polls. */
        const TickType_t current_tick = xTaskGetTickCount();
        if (current_tick - next_wake_tick > polling_period) next_wake_tick = current_tick;
#endif
        vTaskDelayUntil(&next_wake_tick, polling_period);
    }
    atomic_store(&s_acquisition_active, false);
    vTaskDelete(NULL);
}

esp_err_t imu_manager_init(void)
{
    s_device_lock = xSemaphoreCreateMutex();
    s_dmp_sample_lock = xSemaphoreCreateMutex();
    s_raw_sample_lock = xSemaphoreCreateMutex();
    if (!s_device_lock || !s_dmp_sample_lock || !s_raw_sample_lock) {
        if (s_device_lock) vSemaphoreDelete(s_device_lock);
        if (s_dmp_sample_lock) vSemaphoreDelete(s_dmp_sample_lock);
        if (s_raw_sample_lock) vSemaphoreDelete(s_raw_sample_lock);
        s_device_lock = s_dmp_sample_lock = s_raw_sample_lock = NULL;
        ESP_LOGE(TAG, "IMU init: mutex allocation failed: ESP_ERR_NO_MEM");
        return ESP_ERR_NO_MEM;
    }
    SYSTEM_STATE_SET(imu_state, IMU_STATE_INITIALIZING);
#if CONFIG_IMU_ENABLED
    s_startup_retries_left = IMU_STARTUP_RETRY_COUNT;
    const esp_err_t error = configure_sensor(true, true);
    if (error != ESP_OK) {
        schedule_startup_retry(error);
    } else {
        atomic_store(&s_sensor_ready, true);
    }
#endif
    return ESP_OK;
}

esp_err_t imu_manager_start(void)
{
    if (!s_device_lock || s_acquisition_task) return ESP_ERR_INVALID_STATE;
    atomic_store(&s_acquisition_requested, true);
    atomic_store(&s_acquisition_active, true);
    if (xTaskCreate(acquisition_task, "imu", 4096, NULL, 8, &s_acquisition_task) != pdPASS) {
        atomic_store(&s_acquisition_requested, false);
        atomic_store(&s_acquisition_active, false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void imu_manager_stop(void)
{
    atomic_store(&s_acquisition_requested, false);
    /* Wait for in-flight I2C access before the power manager sleeps the sensor. */
    while (atomic_load(&s_acquisition_active)) vTaskDelay(1);
    s_acquisition_task = NULL;
}

esp_err_t imu_manager_sleep(void)
{
    if (!s_sensor_ready) return ESP_OK;
#if CONFIG_IMU_ENABLED
    xSemaphoreTake(s_device_lock, portMAX_DELAY);
    imu_magnetometer_invalidate(&s_magnetometer);
    if (s_runtime_recovery_armed) imu_i2c_begin(&s_i2c, IMU_STREAM_TIMEOUT_MS);
    icm20948_status_e status = icm20948_sleep(&s_sensor, true);
    esp_err_t error = s_i2c.first_error;
    xSemaphoreGive(s_device_lock);
    if (error != ESP_OK) return error;
    return status == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL;
#else
    return ESP_OK;
#endif
}

esp_err_t imu_manager_reset_fifo(void)
{
#if CONFIG_IMU_DMP_ENABLED
    xSemaphoreTake(s_device_lock, portMAX_DELAY);
    if (!atomic_load(&s_sensor_ready)) {
        xSemaphoreGive(s_device_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_dmp_fifo_needs_reset = true;
    const bool recovered = recover_dmp_fifo();
    xSemaphoreGive(s_device_lock);
    return recovered ? ESP_OK : ESP_FAIL;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool imu_manager_latest_dmp(imu_dmp_sample_t *sample)
{
    if (!sample || !s_dmp_sample_lock) return false;
    xSemaphoreTake(s_dmp_sample_lock, portMAX_DELAY);
    *sample = s_latest_dmp_sample;
    xSemaphoreGive(s_dmp_sample_lock);
    return (sample->flags & IMU_FLAG_VALID) != 0;
}

bool imu_manager_latest_raw(imu_raw_sample_t *sample)
{
    if (!sample || !s_raw_sample_lock) return false;
    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    *sample = s_latest_raw_sample;
    xSemaphoreGive(s_raw_sample_lock);
    return (sample->flags & IMU_RAW_FLAG_VALID) != 0;
}
