#include "imu_i2c_transport.h"

#include <string.h>
#include "sdkconfig.h"

#if CONFIG_IMU_ENABLED

enum { DEFAULT_TRANSFER_TIMEOUT_MS = 20, INVALID_REGISTER_BANK = UINT8_MAX };

static void invalidate_banks(icm20948_device_t *sensor)
{
    sensor->_last_bank = INVALID_REGISTER_BANK;
    sensor->_last_mems_bank = INVALID_REGISTER_BANK;
}

static icm20948_status_e transfer_result(imu_i2c_transport_t *transport,
                                         esp_err_t error, uint8_t reg, bool write)
{
    if (error == ESP_OK) return ICM_20948_STAT_OK;
    if (transport->first_error == ESP_OK) {
        transport->first_error = error;
        transport->failed_register = reg;
        transport->failed_write = write;
    }
    /* The managed driver can cache bank writes before their callbacks finish. */
    invalidate_banks(transport->sensor);
    return error == ESP_ERR_INVALID_ARG ? ICM_20948_STAT_PARAM_ERR : ICM_20948_STAT_ERR;
}

static icm20948_status_e write_register(uint8_t reg, uint8_t *data, uint32_t length,
                                        void *user)
{
    imu_i2c_transport_t *transport = user;
    esp_err_t error = transport->first_error;
    if (error == ESP_OK) {
        if (length > INV_MAX_SERIAL_WRITE || (length && !data)) {
            error = ESP_ERR_INVALID_ARG;
        } else if (!transport->device) {
            error = ESP_ERR_INVALID_STATE;
        } else {
            uint8_t tx[INV_MAX_SERIAL_WRITE + 1] = {reg};
            if (length) memcpy(tx + 1, data, length);
            error = i2c_master_transmit(transport->device, tx, length + 1,
                                        transport->timeout_ms);
        }
    }
    return transfer_result(transport, error, reg, true);
}

static icm20948_status_e read_register(uint8_t reg, uint8_t *data, uint32_t length,
                                       void *user)
{
    imu_i2c_transport_t *transport = user;
    esp_err_t error = transport->first_error;
    if (error == ESP_OK) {
        if (!length || !data) {
            error = ESP_ERR_INVALID_ARG;
        } else if (!transport->device) {
            error = ESP_ERR_INVALID_STATE;
        } else {
            error = i2c_master_transmit_receive(transport->device, &reg, 1, data,
                                                length, transport->timeout_ms);
        }
    }
    /* Some legacy calls decode the buffer even when their cumulative status
     * is already an error. Avoid uninitialized or partial register values. */
    if (error != ESP_OK && data && length) memset(data, 0, length);
    return transfer_result(transport, error, reg, false);
}

void imu_i2c_attach(imu_i2c_transport_t *transport, icm20948_device_t *sensor)
{
    transport->sensor = sensor;
    transport->serif = (icm20948_serif_t){
        .write = write_register,
        .read = read_register,
        .user = transport,
    };
    icm20948_link_serif(sensor, &transport->serif);
    /* Any previous fault predates this attachment. The cache now belongs to
     * the sensor state established through the stock transport. */
    transport->first_error = ESP_OK;
    imu_i2c_begin(transport, DEFAULT_TRANSFER_TIMEOUT_MS);
}

void imu_i2c_begin(imu_i2c_transport_t *transport, int timeout_ms)
{
    /* A managed driver function may still have updated its cache after its
     * last callback. Once it returns, clear the cache again before retrying. */
    if (transport->first_error != ESP_OK) invalidate_banks(transport->sensor);
    transport->first_error = ESP_OK;
    transport->failed_register = 0;
    transport->failed_write = false;
    transport->timeout_ms = timeout_ms > 0 ? timeout_ms : DEFAULT_TRANSFER_TIMEOUT_MS;
}

#endif
