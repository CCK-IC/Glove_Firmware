#include "imu_manager.h"

#include <math.h>
#include <stdatomic.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "board/board_config.h"
#include "system_state.h"
#include "icm20948.h"
#include "icm20948_i2c.h"

static const char *TAG = "imu";
#if CONFIG_IMU_ENABLED
static icm20948_device_t s_sensor;
static icm0948_config_i2c_t s_i2c_config;
#endif
static SemaphoreHandle_t s_device_lock;
static SemaphoreHandle_t s_dmp_sample_lock;
static SemaphoreHandle_t s_raw_sample_lock;
static imu_dmp_sample_t s_latest_dmp_sample;
static imu_raw_sample_t s_latest_raw_sample;
static TaskHandle_t s_acquisition_task;
static atomic_bool s_acquisition_requested;
static atomic_bool s_acquisition_active;
static bool s_sensor_ready;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
static uint32_t s_dmp_sequence;
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

static void publish_dmp_sample(const imu_dmp_sample_t *sample)
{
    xSemaphoreTake(s_dmp_sample_lock, portMAX_DELAY);
    s_latest_dmp_sample = *sample;
    xSemaphoreGive(s_dmp_sample_lock);
    SYSTEM_STATE_INCREMENT(dmp_sample_count);
}
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
static void publish_raw_sample(const imu_raw_sample_t *sample)
{
    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    s_latest_raw_sample = *sample;
    xSemaphoreGive(s_raw_sample_lock);
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

static esp_err_t configure_sensor(void)
{
#if CONFIG_IMU_ENABLED
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_IMU_I2C_ADDRESS,
        .scl_speed_hz = CONFIG_BOARD_I2C_FREQUENCY_HZ,
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

    vTaskDelay(pdMS_TO_TICKS(50));
    status = icm20948_sw_reset(&s_sensor);
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "IMU init: icm20948_sw_reset failed: rc=%d (0x%x); continuing",
                 (int)status, (unsigned)status);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    s_sensor._last_bank = 255; /* Reset invalidates the driver's bank cache. */
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
    IMU_INIT_CHECK(icm20948_configure_magnetometer(&s_sensor, AK09916_MODE_CONT_100_HZ),
                   ESP_ERR_NOT_FOUND);

#if CONFIG_IMU_DMP_ENABLED
    IMU_INIT_CHECK(icm20948_init_dmp_sensor_with_defaults(&s_sensor), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_enable_dmp_sensor(
        &s_sensor, INV_ICM20948_SENSOR_ORIENTATION, 1), ESP_FAIL);
    IMU_INIT_CHECK(inv_icm20948_set_dmp_sensor_period(
        &s_sensor, DMP_ODR_Reg_Quat9, 0), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_fifo(&s_sensor, true), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_enable_dmp(&s_sensor, true), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_reset_dmp(&s_sensor), ESP_FAIL);
    IMU_INIT_CHECK(icm20948_reset_fifo(&s_sensor), ESP_FAIL);
    SYSTEM_STATE_SET(imu_dmp, true);
    SYSTEM_STATE_SET(requested_imu_rate_hz, CONFIG_IMU_DMP_RATE_HZ);
    /* These are estimates until the acquisition task measures actual output. */
#if defined(CONFIG_IMU_DMP_MANAGED_RATE_HZ)
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, CONFIG_IMU_DMP_MANAGED_RATE_HZ);
#elif defined(CONFIG_IMU_DMP_OVERRIDE_RATE_HZ)
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, CONFIG_IMU_DMP_OVERRIDE_RATE_HZ);
#else
    SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, 0);
#endif
#endif
#endif
    return ESP_OK;
}

#if CONFIG_IMU_DMP_ENABLED
/* Called with s_device_lock held, as all accesses share the driver's bank cache. */
static bool read_dmp_sample(imu_dmp_sample_t *sample, unsigned *consecutive_errors)
{
    icm_20948_DMP_data_t fifo_data = {0};
    icm20948_status_e status = inv_icm20948_read_dmp_data(&s_sensor, &fifo_data);
    if (status == ICM_20948_STAT_FIFO_NO_DATA_AVAIL ||
        status == ICM_20948_STAT_FIFO_INCOMPLETE_DATA) return false;
    if (status != ICM_20948_STAT_OK && status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
        SYSTEM_STATE_INCREMENT(dropped_dmp_samples);
        if (++*consecutive_errors >= CONFIG_IMU_ERROR_REINIT_COUNT) {
            SYSTEM_STATE_SET(imu_state, IMU_STATE_DEGRADED);
            if (icm20948_reset_fifo(&s_sensor) != ICM_20948_STAT_OK)
                system_state_set_error("imu_fifo", ESP_FAIL);
            SYSTEM_STATE_INCREMENT(imu_reinitializations);
            *consecutive_errors = 0;
        }
        return false;
    }
    *consecutive_errors = 0;
    if ((fifo_data.header & DMP_header_bitmap_Quat9) == 0) return false;

    double q1 = (double)fifo_data.Quat9.Data.Q1 / 1073741824.0;
    double q2 = (double)fifo_data.Quat9.Data.Q2 / 1073741824.0;
    double q3 = (double)fifo_data.Quat9.Data.Q3 / 1073741824.0;
    double q0 = sqrt(fmax(0.0, 1.0 - q1*q1 - q2*q2 - q3*q3));
    const double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    *sample = (imu_dmp_sample_t){
        .sequence = ++s_dmp_sequence,
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        .q0_q30 = quaternion_to_q30(q0),
        .q1_q30 = quaternion_to_q30(q1),
        .q2_q30 = quaternion_to_q30(q2),
        .q3_q30 = quaternion_to_q30(q3),
        .accuracy = (uint16_t)fifo_data.Quat9.Data.Accuracy,
        .flags = IMU_FLAG_VALID | IMU_FLAG_DMP,
    };
    return true;
}
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
static bool acquire_raw_sample(uint64_t now_us, const system_state_t *state)
{
    imu_raw_sample_t sample = {
        .timestamp_ms = (uint32_t)(now_us / 1000ULL),
        .actual_rate_hz = (uint16_t)state->actual_raw_sample_rate_hz,
        .flags = IMU_RAW_FLAG_VALID | IMU_RAW_FLAG_ACCEL_VALID |
                 IMU_RAW_FLAG_GYRO_VALID | IMU_RAW_FLAG_MAG_VALID | IMU_RAW_FLAG_TEMP_VALID,
    };
#if CONFIG_IMU_RAW_DUMMY
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
    icm20948_status_e status = icm20948_get_agmt(&s_sensor, &sensor_data);
    xSemaphoreGive(s_device_lock);
    if (status != ICM_20948_STAT_OK) {
        SYSTEM_STATE_INCREMENT(dropped_raw_samples);
        return false;
    }
    sample.sequence = ++s_raw_sequence;
    sample.accel_x = sensor_data.acc.axes.x;
    sample.accel_y = sensor_data.acc.axes.y;
    sample.accel_z = sensor_data.acc.axes.z;
    sample.gyro_x = sensor_data.gyr.axes.x;
    sample.gyro_y = sensor_data.gyr.axes.y;
    sample.gyro_z = sensor_data.gyr.axes.z;
    sample.mag_x = sensor_data.mag.axes.x;
    sample.mag_y = sensor_data.mag.axes.y;
    sample.mag_z = sensor_data.mag.axes.z;
    sample.temperature = sensor_data.tmp.val;
    sample.accel_range_code = sensor_data.fss.a;
    sample.gyro_range_code = sensor_data.fss.g;
    if (state->imu_dmp) {
        /* DMP changes SLV0 to its reserved-register, byte-swapped layout.
         * get_agmt() decodes the normal ST1/XYZ/ST2 layout, so its magnetic
         * values and status bits cannot be trusted in this mode. */
        sample.flags &= (uint8_t)~IMU_RAW_FLAG_MAG_VALID;
    } else {
        enum { MAG_DATA_READY_MASK = 1U << 0, MAG_OVERFLOW_MASK = 1U << 3 };
        if ((sensor_data.magStat1 & MAG_DATA_READY_MASK) == 0)
            sample.flags &= (uint8_t)~IMU_RAW_FLAG_MAG_VALID;
        if ((sensor_data.magStat2 & MAG_OVERFLOW_MASK) != 0) {
            sample.flags |= IMU_RAW_FLAG_MAG_OVERFLOW;
            sample.flags &= (uint8_t)~IMU_RAW_FLAG_MAG_VALID;
        }
    }
#endif
    if (state->imu_dmp) sample.flags |= IMU_RAW_FLAG_DMP_CONCURRENT;
    publish_raw_sample(&sample);
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
#if CONFIG_IMU_DMP_ENABLED
        if (s_sensor_ready) {
            xSemaphoreTake(s_device_lock, portMAX_DELAY);
            valid = read_dmp_sample(&sample, &consecutive_errors);
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
            publish_dmp_sample(&sample);
            ++dmp_sample_count;
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
            SYSTEM_STATE_SET(actual_dmp_sample_rate_hz, dmp_sample_count * 1000000ULL / elapsed_us);
            const uint32_t measured_raw_rate_hz = raw_sample_count * 1000000ULL / elapsed_us;
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
    const esp_err_t error = configure_sensor();
#if CONFIG_IMU_ENABLED
    s_sensor_ready = error == ESP_OK;
#endif
    if (error != ESP_OK) {
#if CONFIG_IMU_ENABLED
        if (s_i2c_config.dev_handle) {
            i2c_master_bus_rm_device(s_i2c_config.dev_handle);
            s_i2c_config.dev_handle = NULL;
        }
#endif
        system_state_set_error("imu", error);
        SYSTEM_STATE_SET(imu_state, IMU_STATE_DEGRADED);
#if CONFIG_IMU_SYNTHETIC_FALLBACK
        ESP_LOGW(TAG, "Sensor unavailable; using marked synthetic data");
        return ESP_OK;
#else
        return error;
#endif
    }
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
    icm20948_status_e status = icm20948_sleep(&s_sensor, true);
    xSemaphoreGive(s_device_lock);
    return status == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL;
#else
    return ESP_OK;
#endif
}

esp_err_t imu_manager_reset_fifo(void)
{
    if (!s_sensor_ready) return ESP_ERR_INVALID_STATE;
#if CONFIG_IMU_DMP_ENABLED
    xSemaphoreTake(s_device_lock, portMAX_DELAY);
    icm20948_status_e status = icm20948_reset_fifo(&s_sensor);
    xSemaphoreGive(s_device_lock);
    SYSTEM_STATE_INCREMENT(imu_reinitializations);
    return status == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL;
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
