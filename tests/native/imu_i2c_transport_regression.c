#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "imu/imu_i2c_transport.h"

static unsigned bus_calls;
static esp_err_t next_read_error;
static esp_err_t next_write_error;
static int expected_timeout_ms;
static int mock_device;

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device, const uint8_t *data,
                              size_t length, int timeout_ms)
{
    assert(device == &mock_device && length > 0 && data);
    assert(timeout_ms == expected_timeout_ms);
    ++bus_calls;
    esp_err_t result = next_write_error;
    next_write_error = ESP_OK;
    return result;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
                                      const uint8_t *write_data, size_t write_length,
                                      uint8_t *read_data, size_t read_length, int timeout_ms)
{
    assert(device == &mock_device && write_data && write_length == 1);
    assert(read_data && read_length > 0 && timeout_ms == expected_timeout_ms);
    ++bus_calls;
    memset(read_data, 0xA5, read_length);  /* Simulate partial/undefined data on error. */
    if (*write_data == AGB0_REG_WHO_AM_I) read_data[0] = ICM_20948_WHOAMI;
    esp_err_t result = next_read_error;
    next_read_error = ESP_OK;
    return result;
}

int main(void)
{
    imu_i2c_transport_t transport = {
        .device = &mock_device,
        .first_error = ESP_ERR_TIMEOUT,
        .failed_register = AGB0_REG_PWR_MGMT_1,
        .failed_write = true,
    };
    icm20948_device_t sensor;
    assert(icm20948_init_struct(&sensor) == ICM_20948_STAT_OK);
    assert(icm20948_link_serif(&sensor, &NullSerif) == ICM_20948_STAT_OK);
    sensor._last_bank = 2;
    sensor._last_mems_bank = 9;
    sensor._dmp_firmware_available = !!CONFIG_ICM_20948_USE_DMP;
    sensor._firmware_loaded = sensor._dmp_firmware_available;
    sensor._enabled_Android_0 = sensor._firmware_loaded ? 0x12 : 0;
    icm20948_device_t before;
    memcpy(&before, &sensor, sizeof(before));

    imu_i2c_attach(&transport, &sensor);
    assert(sensor._serif == &transport.serif);
    before._serif = &transport.serif;
    assert(memcmp(&sensor, &before, sizeof(sensor)) == 0 && bus_calls == 0);
    assert(transport.device == &mock_device && transport.first_error == ESP_OK);
    assert(!transport.failed_register && !transport.failed_write);

    expected_timeout_ms = 30;
    imu_i2c_begin(&transport, expected_timeout_ms);
    uint8_t id = 0;
    assert(icm20948_get_who_am_i(&sensor, &id) == ICM_20948_STAT_OK);
    assert(id == ICM_20948_WHOAMI && bus_calls == 2 && transport.first_error == ESP_OK);

    /* The real managed AGMT routine still visits later driver calls on a
     * timeout. It must avoid further I2C transactions and reading junk bytes. */
    imu_i2c_begin(&transport, expected_timeout_ms);
    next_read_error = ESP_ERR_TIMEOUT;
    sensor._last_mems_bank = 7;
    icm20948_agmt_t sample;
    memset(&sample, 0xFF, sizeof(sample));
    assert(icm20948_get_agmt(&sensor, &sample) != ICM_20948_STAT_OK);
    assert(bus_calls == 3 && transport.first_error == ESP_ERR_TIMEOUT);
    assert(!transport.failed_write && transport.failed_register == AGB0_REG_ACCEL_XOUT_H);
    assert(!sample.acc.axes.x && !sample.gyr.axes.y && !sample.tmp.val && !sample.mag.axes.z);
    assert(sensor._last_bank == UINT8_MAX && sensor._last_mems_bank == UINT8_MAX);

    /* Start a fresh operation after a caller's optimistic post-error cache update. */
    sensor._last_bank = sensor._last_mems_bank = 2;
    imu_i2c_begin(&transport, expected_timeout_ms);
    assert(sensor._last_bank == UINT8_MAX && sensor._last_mems_bank == UINT8_MAX);
    next_write_error = ESP_ERR_INVALID_RESPONSE;
    assert(icm20948_set_bank(&sensor, 2) != ICM_20948_STAT_OK);
    assert(bus_calls == 4 && transport.first_error == ESP_ERR_INVALID_RESPONSE);
    assert(transport.failed_write && transport.failed_register == REG_BANK_SEL);
    id = 0xFF;
    assert(icm20948_execute_r(&sensor, AGB0_REG_WHO_AM_I, &id, 1) != ICM_20948_STAT_OK);
    assert(!id && bus_calls == 4 && transport.first_error == ESP_ERR_INVALID_RESPONSE);

    imu_i2c_begin(&transport, expected_timeout_ms);
    assert(icm20948_get_who_am_i(&sensor, &id) == ICM_20948_STAT_OK);
    assert(id == ICM_20948_WHOAMI && bus_calls == 6 && transport.first_error == ESP_OK);
    puts("PASS: runtime attachment preserves configured state and caches; I2C errors fence the failed operation");
    return 0;
}
