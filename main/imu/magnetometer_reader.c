#include "magnetometer_reader.h"

#include <string.h>
#include "ak09916_registers.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "imu_mag";
enum { STARTUP_BYTE_TIMEOUT_US = 50000, STREAM_BYTE_TIMEOUT_US = 5000,
       STREAM_FRAME_TIMEOUT_US = 20000, STREAM_STATUS_RETRY_US = 100,
       STARTUP_ATTEMPTS = 3 };

/* SLV4 is a one-byte transaction engine. Check every host transfer and bound
 * its completion polling rather than relying on the driver's cycle counter. */
static icm20948_status_e read_register(icm20948_device_t *device, uint8_t bank,
                                      uint8_t reg, uint8_t *value)
{
    icm20948_status_e status = icm20948_set_bank(device, bank);
    if (status == ICM_20948_STAT_OK)
        status = icm20948_execute_r(device, reg, value, 1);
    if (status != ICM_20948_STAT_OK) device->_last_bank = 255;
    return status;
}

static icm20948_status_e write_registers(icm20948_device_t *device, uint8_t bank,
                                        uint8_t reg, uint8_t *values, unsigned count)
{
    icm20948_status_e status = icm20948_set_bank(device, bank);
    if (status == ICM_20948_STAT_OK)
        status = icm20948_execute_w(device, reg, values, count);
    if (status != ICM_20948_STAT_OK) device->_last_bank = 255;
    return status;
}

static icm20948_status_e write_register(icm20948_device_t *device, uint8_t bank,
                                       uint8_t reg, uint8_t value)
{
    return write_registers(device, bank, reg, &value, 1);
}

/* Confirm that a failed transaction cannot overlap a retry. */
static icm20948_status_e quiesce_transaction(icm20948_device_t *device)
{
    icm20948_status_e status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0);
    if (status != ICM_20948_STAT_OK) return status;
    uint8_t control = 0;
    status = read_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, &control);
    if (status != ICM_20948_STAT_OK) return status;
    return control & 0x80 ? ICM_20948_STAT_ERR : ICM_20948_STAT_OK;
}

static icm20948_status_e magnetometer_byte(icm20948_device_t *device, uint8_t reg,
    uint8_t *value, bool read, int64_t acquisition_deadline, bool drain_status)
{
    enum { SLV4_DONE = 0x40, SLV4_NACK = 0x10, LOST_ARBITRATION = 0x20 };
    const bool streaming = acquisition_deadline != 0;
    const int64_t started_us = esp_timer_get_time();
    int64_t deadline = started_us + (streaming ? STREAM_BYTE_TIMEOUT_US : STARTUP_BYTE_TIMEOUT_US);
    if (streaming && acquisition_deadline < deadline)
        deadline = acquisition_deadline;
    const char *phase = "setup deadline";
    bool triggered = false;
    uint8_t master_status = 0;
    icm20948_status_e status = ICM_20948_STAT_ERR;
    if (esp_timer_get_time() >= deadline) goto failed;
    if (drain_status) {
        phase = "clear status";
        status = read_register(device, 0, AGB0_REG_I2C_MST_STATUS, &master_status);
        if (status != ICM_20948_STAT_OK) goto failed;
    }
    phase = "slave address";
    const uint8_t address = MAG_AK09916_I2C_ADDR | (read ? 0x80 : 0);
    if (streaming) {
        uint8_t setup[] = { address, reg };
        status = write_registers(device, 3, AGB3_REG_I2C_PERIPH4_ADDR, setup, sizeof(setup));
        if (status != ICM_20948_STAT_OK) goto failed;
    } else {
        status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_ADDR, address);
        if (status != ICM_20948_STAT_OK) goto failed;
        phase = "register address";
        status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_REG, reg);
        if (status != ICM_20948_STAT_OK) goto failed;
    }
    if (!read) {
        phase = "data out";
        status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_DO, *value);
        if (status != ICM_20948_STAT_OK) goto failed;
    }
    phase = "setup deadline";
    if (esp_timer_get_time() >= deadline) {
        status = ICM_20948_STAT_ERR;
        goto failed;
    }
    phase = "start transfer";
    triggered = true; /* A failed host write may still have reached hardware. */
    status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0x80);
    if (status != ICM_20948_STAT_OK) goto failed;

    phase = "completion";
    do {
        status = read_register(device, 0, AGB0_REG_I2C_MST_STATUS, &master_status);
        if (status != ICM_20948_STAT_OK) goto failed;
        if (master_status & (SLV4_NACK | LOST_ARBITRATION)) {
            status = ICM_20948_STAT_ERR;
            goto failed;
        }
        if (master_status & SLV4_DONE) {
            phase = "data in";
            if (read)
                status = read_register(device, 3, AGB3_REG_I2C_PERIPH4_DI, value);
            if (status != ICM_20948_STAT_OK) goto failed;
            if (esp_timer_get_time() >= deadline) {
                status = ICM_20948_STAT_ERR;
                goto failed;
            }
            return ICM_20948_STAT_OK;
        }
        if (streaming) {
            /* The auxiliary transfer runs on the sensor's clock. Leave time
             * for it to finish before issuing another host transaction. */
            const int64_t remaining_us = deadline - esp_timer_get_time();
            if (remaining_us <= 0) break;
            esp_rom_delay_us((uint32_t)(remaining_us < STREAM_STATUS_RETRY_US ?
                                       remaining_us : STREAM_STATUS_RETRY_US));
        }
    } while (esp_timer_get_time() < deadline);
    status = ICM_20948_STAT_ERR;

failed:
    if (triggered) (void)write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0);
    /* Streaming errors are reported by the manager; avoid logging per packet. */
    if (!streaming)
        ESP_LOGE(TAG, "SLV4 %s reg=0x%02x phase=%s rc=%d mst=0x%02x elapsed=%lld us",
                 read ? "read" : "write", reg, phase, (int)status, master_status,
                 (long long)(esp_timer_get_time() - started_us));
    return status;
}

/* Keep the exact failing operation visible even when the driver returns ERR=1. */
#define MAG_CONFIG_CHECK(call) do { \
    icm20948_status_e config_rc_ = (call); \
    if (config_rc_ != ICM_20948_STAT_OK) { \
        device->_last_bank = 255; \
        ESP_LOGE(TAG, "Configure: %s failed rc=%d", #call, (int)config_rc_); \
        return config_rc_; \
    } \
} while (0)

static icm20948_status_e probe_identity(icm20948_device_t *device)
{
    icm20948_status_e status = ICM_20948_STAT_ERR;
    for (unsigned attempt = 0; attempt < STARTUP_ATTEMPTS; ++attempt) {
        uint8_t identity1 = 0, identity2 = 0;
        status = magnetometer_byte(device, AK09916_REG_WIA1, &identity1, true,
                                   0, true);
        if (status == ICM_20948_STAT_OK)
            status = magnetometer_byte(device, AK09916_REG_WIA2, &identity2, true,
                                       0, true);
        if (status == ICM_20948_STAT_OK) {
            if ((((uint16_t)identity1 << 8) | identity2) == MAG_AK09916_WHO_AM_I) {
                ESP_LOGI(TAG, "AK09916 identity=0x%02x%02x attempt=%u",
                         identity1, identity2, attempt + 1);
                return ICM_20948_STAT_OK;
            }
            status = ICM_20948_STAT_WRONG_ID;
        }
        ESP_LOGW(TAG, "Identity attempt %u/%u rc=%d WIA1=0x%02x WIA2=0x%02x expected=0x4809",
                 attempt + 1, STARTUP_ATTEMPTS, (int)status, identity1, identity2);
        if (attempt + 1 == STARTUP_ATTEMPTS ||
            (status != ICM_20948_STAT_ERR && status != ICM_20948_STAT_WRONG_ID))
            return status;
        /* The auxiliary master can remain unresponsive after an ICM reset.
         * Reset only that engine, then re-enable its clock and retry identity. */
        device->_last_bank = 255;
        MAG_CONFIG_CHECK(icm20948_i2c_master_enable(device, false));
        MAG_CONFIG_CHECK(icm20948_i2c_master_reset(device));
        esp_rom_delay_us(10000);
        device->_last_bank = 255;
        MAG_CONFIG_CHECK(icm20948_i2c_master_enable(device, true));
        MAG_CONFIG_CHECK(quiesce_transaction(device));
    }
    return status;
}

icm20948_status_e imu_magnetometer_configure(icm20948_device_t *device,
                                            imu_magnetometer_t *mag, bool dmp)
{
    memset(mag, 0, sizeof(*mag));
    mag->needs_recovery = true;
    MAG_CONFIG_CHECK(icm20948_i2c_master_enable(device, true));
    /* SLV0 data reads would consume DRDY before this reader could observe it.
     * Disable SLV1 too, as DMP profiles use it for recurring mode writes. */
    for (unsigned slot = 0; slot < 2; ++slot) {
        MAG_CONFIG_CHECK(icm20948_i2c_controller_configure_peripheral(device, slot,
            MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 0, true, false,
            false, false, false, 0));
    }
    /* Keep the auxiliary ODR scheduler enabled for SLV4. On the tested C3,
     * continuous master mode leaves EN set without DONE, NACK or arbitration. */
    MAG_CONFIG_CHECK(icm20948_set_sample_mode(device, ICM_20948_INTERNAL_MST,
                                            SAMPLE_MODE_CYCLED));
    MAG_CONFIG_CHECK(quiesce_transaction(device));
    MAG_CONFIG_CHECK(probe_identity(device));
    uint8_t reset = 1;
    MAG_CONFIG_CHECK(magnetometer_byte(device, AK09916_REG_CNTL3, &reset, false,
                                       0, true));
    esp_rom_delay_us(1000);
    uint8_t mode = dmp ? 0 : AK09916_MODE_CONT_100_HZ;
    MAG_CONFIG_CHECK(magnetometer_byte(device, AK09916_REG_CNTL2, &mode, false,
                                       0, true));
    uint8_t actual_mode = 0;
    MAG_CONFIG_CHECK(magnetometer_byte(device, AK09916_REG_CNTL2, &actual_mode, true,
                                       0, true));
    if (actual_mode != mode) {
        ESP_LOGE(TAG, "Mode mismatch CNTL2=0x%02x expected=0x%02x", actual_mode, mode);
        return ICM_20948_STAT_ERR;
    }
    mag->needs_recovery = false;
    ESP_LOGI(TAG, "Configured %s mode CNTL2=0x%02x", dmp ? "DMP power-down" : "raw 100Hz", actual_mode);
    return ICM_20948_STAT_OK;
}

void imu_magnetometer_update(imu_magnetometer_t *mag, int16_t x, int16_t y,
                             int16_t z, uint64_t now_us)
{
    mag->x = x;
    mag->y = y;
    mag->z = z;
    mag->updated_at_us = now_us;
    mag->valid = true;
    mag->overflow = false;
}

void imu_magnetometer_invalidate(imu_magnetometer_t *mag)
{
    mag->valid = false;
    mag->overflow = false;
}

icm20948_status_e imu_magnetometer_poll(icm20948_device_t *device,
                                       imu_magnetometer_t *mag, uint64_t now_us)
{
    if (now_us < mag->next_poll_us) return ICM_20948_STAT_OK;
    mag->next_poll_us = now_us + IMU_MAG_POLL_PERIOD_US;
    const int64_t deadline = esp_timer_get_time() +
        (mag->frame_timeout_us ? mag->frame_timeout_us : STREAM_FRAME_TIMEOUT_US);
    uint8_t status1 = 0, status2 = 0;
    bool drain_status = true;
    icm20948_status_e status;
    if (mag->needs_recovery) {
        status = quiesce_transaction(device);
        if (status != ICM_20948_STAT_OK) goto failed;
        mag->needs_recovery = false;
    }
    if (mag->needs_release) {
        /* A failed partial read may leave AK09916's data registers protected. */
        status = magnetometer_byte(device, AK09916_REG_ST2, &status2, true, deadline, true);
        if (status != ICM_20948_STAT_OK) goto failed;
        mag->needs_release = false;
        drain_status = false;
    }
    status = magnetometer_byte(device, AK09916_REG_ST1, &status1, true, deadline, drain_status);
    if (status != ICM_20948_STAT_OK) goto failed;
    if (!(status1 & 0x01)) return ICM_20948_STAT_OK;

    /* DONE reads clear the master's status. Under the device lock no other
     * SLV4 transaction can run between these reads. */
    uint8_t data[6]; /* HXL..HZH */
    mag->needs_release = true;
    for (unsigned i = 0; i < sizeof(data); ++i) {
        status = magnetometer_byte(device, AK09916_REG_HXL + i, &data[i], true, deadline, false);
        if (status != ICM_20948_STAT_OK) goto failed;
    }
    /* ST2 releases the data latch. TMPS is a dummy register. */
    status = magnetometer_byte(device, AK09916_REG_ST2, &status2, true, deadline, false);
    if (status != ICM_20948_STAT_OK) goto failed;
    mag->needs_release = false;
    if (status2 & 0x08) {
        mag->updated_at_us = now_us;
        mag->valid = false;
        mag->overflow = true;
        return ICM_20948_STAT_OK;
    }
    const int16_t x = (int16_t)((uint16_t)data[0] | (uint16_t)data[1] << 8);
    const int16_t y = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8);
    const int16_t z = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
    imu_magnetometer_update(mag, x, y, z, now_us);
    return ICM_20948_STAT_OK;

failed:
    mag->needs_recovery = true;
    imu_magnetometer_invalidate(mag);
    return status;
}

void imu_magnetometer_apply(const imu_magnetometer_t *mag,
                            imu_raw_sample_t *sample, uint64_t now_us)
{
    sample->mag_x = mag->x;
    sample->mag_y = mag->y;
    sample->mag_z = mag->z;
    sample->flags &= (uint8_t)~(IMU_RAW_FLAG_MAG_VALID | IMU_RAW_FLAG_MAG_OVERFLOW);
    if (mag->overflow) sample->flags |= IMU_RAW_FLAG_MAG_OVERFLOW;
    if (mag->valid && now_us >= mag->updated_at_us &&
        now_us - mag->updated_at_us < IMU_MAG_FRESHNESS_US)
        sample->flags |= IMU_RAW_FLAG_MAG_VALID;
}
