#include "magnetometer_reader.h"

#include <string.h>
#include "ak09916_registers.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

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

static icm20948_status_e write_register(icm20948_device_t *device, uint8_t bank,
                                       uint8_t reg, uint8_t value)
{
    icm20948_status_e status = icm20948_set_bank(device, bank);
    if (status == ICM_20948_STAT_OK)
        status = icm20948_execute_w(device, reg, &value, 1);
    if (status != ICM_20948_STAT_OK) device->_last_bank = 255;
    return status;
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
                                          uint8_t *value, bool read, int64_t acquisition_deadline)
{
    enum { SLV4_DONE = 0x40, SLV4_NACK = 0x10, LOST_ARBITRATION = 0x20 };
    int64_t deadline = esp_timer_get_time() + 5000;
    if (acquisition_deadline && acquisition_deadline < deadline)
        deadline = acquisition_deadline;
    if (esp_timer_get_time() >= deadline) return ICM_20948_STAT_ERR;
    uint8_t master_status = 0;
    icm20948_status_e status = read_register(device, 0, AGB0_REG_I2C_MST_STATUS,
                                             &master_status);
    if (status != ICM_20948_STAT_OK) return status;
    status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_ADDR,
                            MAG_AK09916_I2C_ADDR | (read ? 0x80 : 0));
    if (status != ICM_20948_STAT_OK) return status;
    status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_REG, reg);
    if (status != ICM_20948_STAT_OK) return status;
    if (!read) {
        status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_DO, *value);
        if (status != ICM_20948_STAT_OK) return status;
    }
    if (esp_timer_get_time() >= deadline) return ICM_20948_STAT_ERR;
    status = write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0x80);
    if (status != ICM_20948_STAT_OK) {
        /* A host error can occur after the auxiliary engine has started. */
        (void)write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0);
        return status;
    }

    do {
        status = read_register(device, 0, AGB0_REG_I2C_MST_STATUS, &master_status);
        if (status != ICM_20948_STAT_OK) break;
        if (master_status & (SLV4_NACK | LOST_ARBITRATION)) {
            status = ICM_20948_STAT_ERR;
            break;
        }
        if (master_status & SLV4_DONE) {
            if (read)
                status = read_register(device, 3, AGB3_REG_I2C_PERIPH4_DI, value);
            if (esp_timer_get_time() >= deadline) return ICM_20948_STAT_ERR;
            return status;
        }
    } while (esp_timer_get_time() < deadline);

    /* Cancel the outstanding byte before a later retry. */
    (void)write_register(device, 3, AGB3_REG_I2C_PERIPH4_CTRL, 0);
    return status == ICM_20948_STAT_OK ? ICM_20948_STAT_ERR : status;
}

icm20948_status_e imu_magnetometer_configure(icm20948_device_t *device,
                                            imu_magnetometer_t *mag, bool dmp)
{
    memset(mag, 0, sizeof(*mag));
    mag->needs_recovery = true;
    icm20948_status_e status = icm20948_i2c_master_enable(device, true);
    if (status != ICM_20948_STAT_OK) return status;
    /* SLV0 data reads would consume DRDY before this reader could observe it.
     * Disable SLV1 too, as DMP profiles use it for recurring mode writes. */
    for (unsigned slot = 0; slot < 2; ++slot) {
        status = icm20948_i2c_controller_configure_peripheral(device, slot,
            MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 0, true, false,
            false, false, false, 0);
        if (status != ICM_20948_STAT_OK) return status;
    }
    status = icm20948_set_sample_mode(device, ICM_20948_INTERNAL_MST,
                                     SAMPLE_MODE_CONTINUOUS);
    if (status != ICM_20948_STAT_OK) return status;
    status = quiesce_transaction(device);
    if (status != ICM_20948_STAT_OK) return status;
    uint8_t identity1 = 0, identity2 = 0;
    status = magnetometer_byte(device, AK09916_REG_WIA1, &identity1, true, 0);
    if (status != ICM_20948_STAT_OK) return status;
    status = magnetometer_byte(device, AK09916_REG_WIA2, &identity2, true, 0);
    if (status != ICM_20948_STAT_OK) return status;
    if ((((uint16_t)identity1 << 8) | identity2) != MAG_AK09916_WHO_AM_I)
        return ICM_20948_STAT_ERR;

    uint8_t reset = 1;
    status = magnetometer_byte(device, AK09916_REG_CNTL3, &reset, false, 0);
    if (status != ICM_20948_STAT_OK) return status;
    esp_rom_delay_us(100);
    uint8_t mode = dmp ? 0 : AK09916_MODE_CONT_100_HZ;
    status = magnetometer_byte(device, AK09916_REG_CNTL2, &mode, false, 0);
    if (status != ICM_20948_STAT_OK) return status;
    uint8_t actual_mode = 0;
    status = magnetometer_byte(device, AK09916_REG_CNTL2, &actual_mode, true, 0);
    if (status != ICM_20948_STAT_OK) return status;
    if (actual_mode != mode) return ICM_20948_STAT_ERR;
    mag->needs_recovery = false;
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
    const int64_t deadline = esp_timer_get_time() + 20000;
    uint8_t status1 = 0, status2 = 0;
    icm20948_status_e status;
    if (mag->needs_recovery) {
        status = quiesce_transaction(device);
        if (status != ICM_20948_STAT_OK) goto failed;
        mag->needs_recovery = false;
    }
    if (mag->needs_release) {
        /* A failed partial read may leave AK09916's data registers protected. */
        status = magnetometer_byte(device, AK09916_REG_ST2, &status2, true, deadline);
        if (status != ICM_20948_STAT_OK) goto failed;
        mag->needs_release = false;
    }
    status = magnetometer_byte(device, AK09916_REG_ST1, &status1, true, deadline);
    if (status != ICM_20948_STAT_OK) goto failed;
    if (!(status1 & 0x01)) return ICM_20948_STAT_OK;

    uint8_t data[8]; /* HXL..HZH, TMPS, ST2 */
    mag->needs_release = true;
    for (unsigned i = 0; i < sizeof(data); ++i) {
        status = magnetometer_byte(device, AK09916_REG_HXL + i, &data[i], true, deadline);
        if (status != ICM_20948_STAT_OK) goto failed;
    }
    mag->needs_release = false;
    if (data[7] & 0x08) {
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
