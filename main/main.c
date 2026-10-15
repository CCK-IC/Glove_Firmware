#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_agmt"
typedef struct {
	uint32_t sequence;
	uint32_t timestamp_ms;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;

	int16_t temperature;

	uint16_t actual_rate_hz;

	uint8_t flags;
	uint8_t accel_range_code;
	uint8_t gyro_range_code;
	uint8_t mag_status1;
	uint8_t mag_status2;
} imu_sample_t;

/* ICM 20948 configuration */
static icm20948_device_t icm;
static icm0948_config_i2c_t icm_config;
static imu_sample_t s_latest;
static uint32_t s_raw_sequence;

static TaskHandle_t s_task;
static SemaphoreHandle_t s_sample_lock;
void imu_task(void *arg);

static const char * const STATE_STRINGS[] = {
    [ICM_20948_STAT_OK] = "OK",
    [ICM_20948_STAT_ERR] = "Error",
    [ICM_20948_STAT_NOT_IMPL] = "Not Implemented",
    [ICM_20948_STAT_PARAM_ERR] = "Parameter Error",
    [ICM_20948_STAT_WRONG_ID] = "Wrong ID",
    [ICM_20948_STAT_INVALID_SENSOR] = "Invalid Sensor",
    [ICM_20948_STAT_NO_DATA] = "No Data",
    [ICM_20948_STAT_SENSOR_NOT_SUPPORTED] = "Sensor Not Supported",
    [ICM_20948_STAT_DMP_NOT_SUPPORTED] = "DMP Not Supported",
    [ICM_20948_STAT_DMP_VERIFY_FAIL] = "DMP Verify Fail",
    [ICM_20948_STAT_FIFO_NO_DATA_AVAIL] = "FIFO No Data Available",
    [ICM_20948_STAT_FIFO_INCOMPLETE_DATA] = "FIFO Incomplete Data",
    [ICM_20948_STAT_FIFO_MORE_DATA_AVAIL] = "FIFO More Data Available",
    [ICM_20948_STAT_UNRECOGNISED_DMP_HEADER] = "Unrecognised DMP Header",
    [ICM_20948_STAT_UNRECOGNISED_DMP_HEADER_2] = "Unrecognised DMP Header 2",
    [ICM_20948_STAT_INVALID_DMP_REGISTER] = "Invalid DMP Register"
};

const char* state_to_string(icm20948_status_e state) {
    if (state < 0 || state >= ICM_20948_STAT_NUM) {
        return "UNKNOWN_STATE";
    }
    return STATE_STRINGS[state];
}


void print_agmt(icm20948_agmt_t agmt, uint8_t sample_rate_hz)
{
  	ESP_LOGI(TAG, "Acc: [ %+06d, %+06d, %+06d ] Gyr: [ %+06d, %+06d, %+06d ] Mag: [ %+06d, %+06d, %+06d ] MagStatus: [ %s, %s, %s ] Temp: [ %d ] Sample Rate: [ %03d Hz ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		(agmt.magStat1)&0b00000010?"Y":"N",
		(agmt.magStat1)&0b00000001?"Y":"N", 
		(agmt.magStat2)&0b00001000?"Y":"N",
		agmt.tmp.val,
		sample_rate_hz
	);
}

static void publish(const imu_sample_t *sample)
{
    xSemaphoreTake(s_sample_lock, portMAX_DELAY);
    s_latest = *sample;
    xSemaphoreGive(s_sample_lock);

}

void app_main(void)
{

	/* setup i2c */
	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
		.sda_io_num = CONFIG_I2C_MASTER_SDA,
		.scl_io_num = CONFIG_I2C_MASTER_SCL,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

	i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = ICM_20948_I2C_ADDR_AD0,
		.scl_speed_hz = 400000,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &device_config, &icm_config.dev_handle));
	
	/* setup ICM20948 device */
	icm20948_init_i2c(&icm, &icm_config);
		
	/* check ID */
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
	{
		ESP_LOGE(TAG, "check id failed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* check whoami */
	icm20948_status_e stat = ICM_20948_STAT_ERR;
	uint8_t whoami = 0x00;
	do {
		whoami = 0x00;
		stat = icm20948_get_who_am_i(&icm, &whoami);
		if ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI)) {
			ESP_LOGE(TAG, "whoami does not match (0x%02X). Retrying...", whoami);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	} while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI));

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	icm20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS); 

	// Set full scale ranges for both acc and gyr
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
	icm20948_set_full_scale(&icm, sensors, myfss);

	// Set up DLPF configuration
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	myDLPcfg.g = GYR_D361BW4_N376BW5;
	icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	// Choose whether or not to use DLPF
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

	// Now wake the sensor up
	icm20948_sleep(&icm, false);
	icm20948_low_power(&icm, false);
	if (icm20948_configure_magnetometer(&icm, AK09916_MODE_CONT_100_HZ) != ICM_20948_STAT_OK) {
		ESP_LOGE(TAG, "magnetometer configuration failed");
		return;
	}
	s_sample_lock = xSemaphoreCreateMutex();
    if (!s_sample_lock) ESP_LOGE(TAG, "Failed to create sample lock");
	
	if (xTaskCreate(imu_task, "imu", 4096, NULL, 8, &s_task) != pdPASS) {
		ESP_LOGE(TAG, "Failed to create imu task");
        return;
    }
	
}

void imu_task(void *arg)
{
	TickType_t wake = xTaskGetTickCount();
	uint64_t count_window = esp_timer_get_time();
	uint32_t count = 0;
	uint8_t sample_rate_hz = 0;
	bool failed = false;
	uint32_t count_fail = 0;
    /* loop */

    while(1)
	{
		const uint32_t poll_rate = 125; // Hz
        TickType_t period = pdMS_TO_TICKS(1000U / poll_rate);
        if (!period) period = 1;

		icm20948_agmt_t agmt; // = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
		icm20948_status_e status = icm20948_get_agmt(&icm, &agmt);
		imu_sample_t sample = {
			.sequence = s_raw_sequence,
			.timestamp_ms = esp_timer_get_time() / 1000ULL,
			.accel_x = agmt.acc.axes.x,
			.accel_y = agmt.acc.axes.y,
			.accel_z = agmt.acc.axes.z,
			.gyro_x = agmt.gyr.axes.x,
			.gyro_y = agmt.gyr.axes.y,
			.gyro_z = agmt.gyr.axes.z,
			.mag_x = agmt.mag.axes.x,
			.mag_y = agmt.mag.axes.y,
			.mag_z = agmt.mag.axes.z,
			.mag_status1 = agmt.magStat1,
			.mag_status2 = agmt.magStat2,
			.temperature = agmt.tmp.val,
			.actual_rate_hz = sample_rate_hz,
			.flags = status == ICM_20948_STAT_OK ? 1 : 0,
		};
		if(!sample.flags) count_fail++;

		publish(&sample);
		count++;
		// if (status == ICM_20948_STAT_OK) {
		// 	if (failed) {
		// 		ESP_LOGI(TAG, "Recovered from error");
		// 		failed = false;
		// 	}
		// } else {
		// 	failed = true;
		// 	ESP_LOGE(TAG, "Uh oh: %s", state_to_string(status));
		// 	print_agmt(agmt, sample_rate_hz);	
		// 	//dispose packet?
		// 	// for(int i = 0 ;i<20 ;i++) {
		// 	// 	icm20948_agmt_t dummy;
		// 	// 	if(icm20948_get_agmt(&icm, &dummy) == ICM_20948_STAT_OK) break;
		// 	// 	ESP_LOGE(TAG, "Recover failed (%d/20)", i+1);
		// 	// }
		// 	// wait??
		// 	//500ms ok 
		// 	//100ms fastest 3ticks to recover
		// 	//250ms? looks fine min losing 25 samples
		// 	// vTaskDelay(pdMS_TO_TICKS(250));
		// 	/* 125HZ recover test */
		// 	vTaskDelay(pdMS_TO_TICKS(100));
			
		// }        
		const uint64_t now = esp_timer_get_time();
        if (now - count_window >= 500000ULL) {

            sample_rate_hz = count;
			ESP_LOGI(TAG, "Sample rate: %d Hz MagStatus1(OVERRUN/RDY): %s/%s MagStatus2(OVERFLOW): %s .5s FailCount: %d",
				 count*2, 
				 (sample.mag_status1)&0b00000010?"Y":"N",
				 (sample.mag_status1)&0b00000001?"Y":"N", 
				 (sample.mag_status2)&0b00001000?"Y":"N",
				 count_fail
				);
            if(sample.flags) count_fail = 0;
            count = 0;
            count_window = now;
        }
		vTaskDelayUntil(&wake, period);
    }
}