#include "imu_manager.h"

#include <math.h>
#include <string.h>
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
static icm20948_device_t s_icm;
static icm0948_config_i2c_t icm_config;
static SemaphoreHandle_t s_sample_lock;
static SemaphoreHandle_t s_raw_sample_lock;
static imu_raw_sample_t s_latest_raw;
static TaskHandle_t s_task;
static volatile bool s_running;
static bool s_sensor_ok;
static uint32_t s_sequence;
static uint32_t s_raw_sequence;
static uint8_t s_accel_range_code;
static uint8_t s_gyro_range_code;

static int32_t q30(double v)
{
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    return (int32_t)llround(v * 1073741824.0);
}

// static void publish(const imu_sample_t *sample)
// {
//     xSemaphoreTake(s_sample_lock, portMAX_DELAY);
//     s_latest = *sample;
//     xSemaphoreGive(s_sample_lock);

//     system_state_t st;
//     system_state_snapshot(&st);
//     st.sample_count++;
//     system_state_replace(&st);
// }

static void publish_raw(const imu_raw_sample_t *sample)
{
    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    s_latest_raw = *sample;
    xSemaphoreGive(s_raw_sample_lock);

    system_state_t st;
    system_state_snapshot(&st);
    st.raw_sample_count++;
    system_state_replace(&st);
}


static void synthetic_sample(imu_dmp_sample_t *out)
{
    const double angle = (double)(s_sequence % 1000) *
                         (2.0 * M_PI / 1000.0);
    out->q0_q30 = q30(cos(angle * 0.5));
    out->q1_q30 = 0;
    out->q2_q30 = 0;
    out->q3_q30 = q30(sin(angle * 0.5));
    out->accuracy = 0;
    out->flags = IMU_FLAG_VALID | IMU_FLAG_SYNTHETIC;
}

static esp_err_t configure_sensor(void)
{
#if CONFIG_IMU_ENABLED
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_IMU_I2C_ADDRESS,
        .scl_speed_hz = CONFIG_BOARD_I2C_FREQUENCY_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(board_i2c_bus(),
                                               &cfg, &icm_config.dev_handle);
    if (err != ESP_OK) return err;

    icm20948_init_i2c(&s_icm, &icm_config);

#if CONFIG_IMU_DMP_ENABLED
    s_icm._dmp_firmware_available = true;
#else
    s_icm._dmp_firmware_available = false;
#endif

    s_icm._firmware_loaded = false; // Initialize _firmware_loaded
    s_icm._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of icm20948_set_bank.
    s_icm._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
    s_icm._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    s_icm._gyroSFpll = 0;
    s_icm._enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
    s_icm._enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
    s_icm._enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
    s_icm._enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-


    uint8_t who = 0;
    esp_err_t rc = icm20948_get_who_am_i(&s_icm, &who);
    for (uint8_t i = 0; i < 10; i++)
    {
        if (rc != ICM_20948_STAT_OK ||
            who != ICM_20948_WHOAMI) {
            ESP_LOGE(TAG, "Trial %d: WHO_AM_I=0x%02x, expected 0x%02x", i, who, ICM_20948_WHOAMI);
        } else {
            ESP_LOGI(TAG, "DMP WHO OK: 0x%02x", who);
            break;
        }
    }
    
    if(rc != ICM_20948_STAT_OK)return ESP_ERR_NOT_FOUND;

    system_state_t st;
    system_state_snapshot(&st);
    st.who_am_i = who;
    st.imu_state = IMU_STATE_INITIALIZING;
    system_state_replace(&st);

    icm20948_sw_reset(&s_icm);
    vTaskDelay(pdMS_TO_TICKS(250));

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	icm20948_set_sample_mode(&s_icm, sensors, SAMPLE_MODE_CONTINUOUS); 

	// Set full scale ranges for both acc and gyr
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
	icm20948_set_full_scale(&s_icm, sensors, myfss);
    s_accel_range_code = 0;
    s_gyro_range_code = 0;

	// Set up DLPF configuration
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	myDLPcfg.g = GYR_D361BW4_N376BW5;
	icm20948_set_dlpf_cfg(&s_icm, sensors, myDLPcfg);

	// Choose whether or not to use DLPF
	icm20948_enable_dlpf(&s_icm, ICM_20948_INTERNAL_ACC, false);
	icm20948_enable_dlpf(&s_icm, ICM_20948_INTERNAL_GYR, false);

	// Now wake the sensor up
	icm20948_sleep(&s_icm, false);
	icm20948_low_power(&s_icm, false);
    if (icm20948_configure_magnetometer(&s_icm, AK09916_MODE_CONT_100_HZ) != ICM_20948_STAT_OK) {
		ESP_LOGE(TAG, "magnetometer configuration failed");
		return ESP_ERR_NOT_FOUND;
	}
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_DMP_ENABLED
    s_accel_range_code = CONFIG_IMU_RAW_DMP_ACCEL_RANGE_CODE;
    s_gyro_range_code = CONFIG_IMU_RAW_DMP_GYRO_RANGE_CODE;
#endif

#if CONFIG_IMU_DMP_ENABLED
    icm20948_status_e rc = icm20948_init_dmp_sensor_with_defaults(&s_icm); //TOBE overided
    if (rc != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "DMP defaults failed: %d", rc);
        return ESP_FAIL;
    }

    //PATCH2
    system_state_snapshot(&st);
    st.imu_dmp = true;
    st.requested_rate_hz = CONFIG_IMU_DMP_RATE_HZ;

    /*
    * This is an expected initial value only. The runtime one-second measurement
    * replaces it once frames arrive.
    */
    #ifdef CONFIG_IMU_DMP_MANAGED_RATE_HZ
    st.actual_dmp_sample_rate_hz = CONFIG_IMU_DMP_MANAGED_RATE_HZ;
    #else
    st.actual_dmp_sample_rate_hz = CONFIG_IMU_DMP_OVERIDE_RATE_HZ;
    #endif
    system_state_replace(&st);
    //PATCH2

    rc = inv_icm20948_enable_dmp_sensor(
        &s_icm, INV_ICM20948_SENSOR_ORIENTATION, 1);
    if (rc != ICM_20948_STAT_OK) return ESP_FAIL;

    rc = inv_icm20948_set_dmp_sensor_period(
        &s_icm, DMP_ODR_Reg_Quat9, 0);
    if (rc != ICM_20948_STAT_OK) return ESP_FAIL;

    if (icm20948_enable_fifo(&s_icm, true) != ICM_20948_STAT_OK ||
        icm20948_enable_dmp(&s_icm, true) != ICM_20948_STAT_OK ||
        icm20948_reset_dmp(&s_icm) != ICM_20948_STAT_OK ||
        icm20948_reset_fifo(&s_icm) != ICM_20948_STAT_OK) {
        return ESP_FAIL;
    }

    system_state_snapshot(&st);
    st.imu_dmp = true;
    system_state_replace(&st);
#endif

    return ESP_OK;
}

static void task_main(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint64_t count_window = esp_timer_get_time();
    uint32_t count = 0;
    // unsigned consecutive_errors = 0;
#if CONFIG_IMU_RAW_ENABLED
    // uint64_t raw_due_us = count_window;
    uint32_t raw_count = 0;
#endif

    while (s_running) {
        system_state_t st;
        system_state_snapshot(&st);
        uint32_t rate = st.requested_rate_hz ? st.requested_rate_hz : 1;
        TickType_t period = pdMS_TO_TICKS(1000 / rate);
        //PATCH2
        // const uint32_t fifo_poll_rate = CONFIG_IMU_FIFO_POLL_RATE_HZ;
        // TickType_t period = pdMS_TO_TICKS(1000U / fifo_poll_rate);
        if (!period) period = 1;

        bool valid = false;
        
#if CONFIG_IMU_DMP_ENABLED
        // imu_sample_t sample = {
        //     .sequence = ++s_sequence,
        //     .timestamp_ms = esp_timer_get_time() / 1000ULL,
        // };
        if (s_sensor_ok) {
            for (unsigned frame = 0; frame < 1; ++frame) {
                icm_20948_DMP_data_t d = {0};

                icm20948_status_e rc =
                    inv_icm20948_read_dmp_data(&s_icm, &d);

                if (rc == ICM_20948_STAT_FIFO_NO_DATA_AVAIL) {
                    break;
                }

                if (rc == ICM_20948_STAT_FIFO_INCOMPLETE_DATA) {
                    /*
                    * Leave the partial frame for a later poll. This is not an I2C
                    * failure and not necessarily a FIFO overflow.
                    */
                    break;
                }

                if (rc != ICM_20948_STAT_OK &&
                    rc != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                    consecutive_errors++;

                    if (consecutive_errors >=
                        CONFIG_IMU_ERROR_REINIT_COUNT) {
                        system_state_snapshot(&st);
                        st.imu_state = IMU_STATE_DEGRADED;
                        system_state_replace(&st);

                        icm20948_status_e reset_rc =
                            icm20948_reset_fifo(&s_icm);

                        if (reset_rc == ICM_20948_STAT_OK) {
                            system_state_snapshot(&st);
                            st.fifo_overflows++;
                            system_state_replace(&st);
                        } else{
                            system_state_snapshot(&st);
                            if (st.fifo_overflows>0) st.fifo_overflows--;
                            system_state_replace(&st);
                        }

                        consecutive_errors = 0;
                    }

                    // break;
                }

                consecutive_errors = 0;

                if ((rc == ICM_20948_STAT_OK) || (rc == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL)) 
		        {
                    if ((d.header & DMP_header_bitmap_Quat9) != 0) {
                        imu_sample_t sample = {
                            .sequence = ++s_sequence,
                            .timestamp_ms =
                                (uint32_t)(esp_timer_get_time() / 1000ULL),
                            .accuracy = (uint16_t)d.Quat9.Data.Accuracy,
                            .flags = IMU_FLAG_VALID | IMU_FLAG_DMP,
                        };
    
                        double q1 =
                            (double)d.Quat9.Data.Q1 / 1073741824.0;
                        double q2 =
                            (double)d.Quat9.Data.Q2 / 1073741824.0;
                        double q3 =
                            (double)d.Quat9.Data.Q3 / 1073741824.0;
    
                        double vector_sum =
                            q1 * q1 + q2 * q2 + q3 * q3;
    
                        double q0 = sqrt(fmax(0.0, 1.0 - vector_sum));
                        double norm = sqrt(
                            q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    
                        if (norm > 0.0) {
                            q0 /= norm;
                            q1 /= norm;
                            q2 /= norm;
                            q3 /= norm;
                        }
    
                        sample.q0_q30 = q30(q0);
                        sample.q1_q30 = q30(q1);
                        sample.q2_q30 = q30(q2);
                        sample.q3_q30 = q30(q3);
    
                        publish(&sample);
                        count++;
                    }
                }


                if (rc != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                    break;
                }
            }
        }
#endif

#if CONFIG_IMU_SYNTHETIC_FALLBACK
        if (!s_sensor_ok) {
            synthetic_sample(&sample);
            valid = true;
        }
#elif CONFIG_IMU_DMP_ENABLED
        if (!s_sensor_ok || !valid) {
            imu_manager_fifo_reinitialize();
        }
#endif

#if CONFIG_IMU_DMP_ENABLED
        if (valid) {
            publish(&sample);
            count++;
        } else {
            system_state_snapshot(&st);
            st.dropped_samples++;
            system_state_replace(&st);
        }
#endif

#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
        uint64_t raw_now_us = esp_timer_get_time();
        if (s_sensor_ok /* && raw_now_us >= raw_due_us */) {
            // remove the raw sample rate limit for now, since we are not using it
            // const uint32_t raw_rate =
            //     CONFIG_IMU_RAW_RATE_HZ ? CONFIG_IMU_RAW_RATE_HZ : 1;
            // raw_due_us = raw_now_us + 1000000ULL / raw_rate;

            icm20948_agmt_t agmt = {0};
#ifndef CONFIG_IMU_RAW_DUMMY
            if (icm20948_get_agmt(&s_icm, &agmt) ==
                ICM_20948_STAT_OK) {
                imu_raw_sample_t raw = {
                    .sequence = ++s_raw_sequence,
                    .timestamp_ms =
                        (uint32_t)(raw_now_us / 1000ULL),
                    .accel_x = (int16_t)agmt.acc.axes.x,
                    .accel_y = (int16_t)agmt.acc.axes.y,
                    .accel_z = (int16_t)agmt.acc.axes.z,
                    .gyro_x = (int16_t)agmt.gyr.axes.x,
                    .gyro_y = (int16_t)agmt.gyr.axes.y,
                    .gyro_z = (int16_t)agmt.gyr.axes.z,
                    .mag_x = (int16_t)agmt.mag.axes.x,
                    .mag_y = (int16_t)agmt.mag.axes.y,
                    .mag_z = (int16_t)agmt.mag.axes.z,
                    .temperature = (int16_t)agmt.tmp.val,
                    .actual_rate_hz =
                        (uint16_t)st.actual_raw_sample_rate_hz,
                    .flags =
                        IMU_RAW_FLAG_VALID |
                        IMU_RAW_FLAG_ACCEL_VALID |
                        IMU_RAW_FLAG_GYRO_VALID |
                        IMU_RAW_FLAG_MAG_VALID |
                        IMU_RAW_FLAG_TEMP_VALID,
                    .accel_range_code = s_accel_range_code,
                    .gyro_range_code = s_gyro_range_code,
                };

                if (st.imu_dmp) {
                    raw.flags |= IMU_RAW_FLAG_DMP_CONCURRENT;
                }
#else

                imu_raw_sample_t raw = {
                    .sequence = ++s_raw_sequence,
                    .timestamp_ms =
                        (uint32_t)(raw_now_us / 1000ULL),
                    .accel_x = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .accel_y = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .accel_z = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .gyro_x = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .gyro_y = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .gyro_z = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .mag_x = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .mag_y = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .mag_z = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .temperature = (int16_t)((s_raw_sequence % 65536) - 32768),
                    .actual_rate_hz =
                        (uint16_t)st.actual_raw_sample_rate_hz,
                    .flags =
                        IMU_RAW_FLAG_VALID |
                        IMU_RAW_FLAG_ACCEL_VALID |
                        IMU_RAW_FLAG_GYRO_VALID |
                        IMU_RAW_FLAG_MAG_VALID |
                        IMU_RAW_FLAG_TEMP_VALID,
                    .accel_range_code = s_accel_range_code,
                    .gyro_range_code = s_gyro_range_code,
                };
#endif

                publish_raw(&raw);
                raw_count++;
            } else {
                system_state_snapshot(&st);
                st.dropped_raw_samples++;
                system_state_replace(&st);
            }
        }
#endif

        const uint64_t now = esp_timer_get_time();
        if (now - count_window >= 1000000ULL) {
            system_state_snapshot(&st);
            st.actual_dmp_sample_rate_hz = count;
            st.imu_state = s_sensor_ok ?
                IMU_STATE_RUNNING : IMU_STATE_DEGRADED;
#if CONFIG_IMU_RAW_ENABLED
            st.actual_raw_sample_rate_hz = raw_count;
#endif
            system_state_replace(&st);
#if CONFIG_IMU_RAW_ENABLED
            xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
            s_latest_raw.actual_rate_hz = (uint16_t)raw_count;
            xSemaphoreGive(s_raw_sample_lock);
            raw_count = 0;
#endif
            
            count = 0;
            count_window = now;
        }

        vTaskDelayUntil(&wake, period);
    }

    vTaskDelete(NULL);
}

esp_err_t imu_manager_init(void)
{
    s_sample_lock = xSemaphoreCreateMutex();
    if (!s_sample_lock) return ESP_ERR_NO_MEM;

    s_raw_sample_lock = xSemaphoreCreateMutex();
    if (!s_raw_sample_lock) {
        vSemaphoreDelete(s_sample_lock);
        s_sample_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    system_state_t st;
    system_state_snapshot(&st);
    st.imu_state = IMU_STATE_INITIALIZING;
    system_state_replace(&st);

    esp_err_t err = configure_sensor();
    s_sensor_ok = err == ESP_OK;

    if (!s_sensor_ok) {
#if CONFIG_IMU_SYNTHETIC_FALLBACK
        ESP_LOGW(TAG, "Sensor unavailable; using marked synthetic data");
        system_state_snapshot(&st);
        st.imu_state = IMU_STATE_DEGRADED;
        system_state_replace(&st);
        return ESP_OK;
#else
        system_state_set_error("imu", err);
        return err;
#endif
    }

    return ESP_OK;
}

esp_err_t imu_manager_start(void)
{
    if (s_task) return ESP_ERR_INVALID_STATE;
    s_running = true;
    if (xTaskCreate(task_main, "imu", 4096, NULL, 8, &s_task) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void imu_manager_stop(void)
{
    s_running = false;
    s_task = NULL;
}

esp_err_t imu_manager_sleep(void)
{
    if (!s_sensor_ok) return ESP_OK;
    return icm20948_sleep(&s_icm, true) == ICM_20948_STAT_OK
        ? ESP_OK : ESP_FAIL;
}

esp_err_t imu_manager_fifo_reinitialize(void)
{
    system_state_t st;
    system_state_snapshot(&st);
    st.imu_reinitializations++;
    system_state_replace(&st);

    if (!s_sensor_ok) return ESP_ERR_INVALID_STATE;
    return icm20948_reset_fifo(&s_icm) == ICM_20948_STAT_OK
        ? ESP_OK : ESP_FAIL;
}

// bool imu_manager_latest(imu_sample_t *sample)
// {
//     if (!sample || !s_sample_lock) return false;
//     xSemaphoreTake(s_sample_lock, portMAX_DELAY);
//     *sample = s_latest;
//     bool valid = (s_latest.flags & IMU_FLAG_VALID) != 0;
//     xSemaphoreGive(s_sample_lock);
//     return valid;
// }

bool imu_manager_latest_raw(imu_raw_sample_t *sample)
{
    if (!sample || !s_raw_sample_lock) return false;

    xSemaphoreTake(s_raw_sample_lock, portMAX_DELAY);
    *sample = s_latest_raw;
    bool valid =
        (s_latest_raw.flags & IMU_RAW_FLAG_VALID) != 0;
    xSemaphoreGive(s_raw_sample_lock);

    return valid;
}