/* Test production code through the actual managed driver's API declarations.
 * The fake models read-to-clear MST status, delayed SLV4 completion, and the
 * AK09916 data latch, and the Nov 11 board's cycled-only auxiliary scheduling.
 * It never supplies a replacement icm20948 header. */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "imu/magnetometer_reader.h"
#include "ak09916_registers.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

enum { DONE = 0x40, NACK = 0x10, LOST_ARB = 0x20, TIMEOUT = 0x100 };
enum { MAG_FLAGS = IMU_RAW_FLAG_MAG_VALID | IMU_RAW_FLAG_MAG_OVERFLOW };
enum { TRACE_CAPACITY = 64, READY_TRANSACTIONS = 8 };
static const uint64_t START = 1000000;
static const icm20948_status_e HOST_ERROR = ICM_20948_STAT_PARAM_ERR;

typedef struct {
    uint8_t reg, value;
    bool read, completed;
} transaction_t;

static struct {
    icm20948_device_t device;
    uint8_t ak[256], addr, reg, dout, din, status;
    uint8_t bank;
    transaction_t trace[TRACE_CAPACITY];
    unsigned transactions, host_calls, fail_host, fault_transaction, fault;
    unsigned reg_reads, reg_writes, status_reads, pending_polls, completion_polls, status_step_us;
    unsigned cancels, delays;
    unsigned ctrl_reads, fail_cancels, fail_ctrl_reads, fail_start_transaction;
    unsigned setup_stall_us, di_stall_us;
    uint8_t stall_reg;
    unsigned master_calls, peripheral_calls[2], sample_mode_calls;
    icm20948_lp_config_cycle_e master_mode;
    unsigned master_disable_calls, master_reset_calls, recovery_delays;
    unsigned resets_needed, persistent_fault, recovery_stage, fail_recovery_api;
    unsigned completion_delay_us, first_completion_delay_us;
    unsigned recovery_host_calls[5];
    icm20948_status_e recovery_api_error;
    int64_t now_us, reset_at_us;
    int64_t master_reset_at_us, transaction_started_at_us;
    bool active, locked, bank_error_pending, bad_mode_readback;
    bool stop_stuck, gate_required, master_enabled, truncate_setup;
} hw;

/* One-shot failures enumerate every host/API seam on the successful path. */
static bool host_step(icm20948_device_t *device, bool bank_access)
{
    assert(device == &hw.device);
    if (hw.bank_error_pending) {
        assert(device->_last_bank == 255);
        hw.bank_error_pending = false;
    }
    assert(++hw.host_calls < 2000); /* Also bounds broken busy loops. */
    if (hw.host_calls != hw.fail_host) return true;
    hw.bank_error_pending = bank_access;
    return false;
}

int64_t esp_timer_get_time(void)
{
    return hw.now_us;
}

void esp_rom_delay_us(uint32_t us)
{
    assert(us > 0);
    ++hw.delays;
    if (hw.recovery_stage == 2 && us >= 10000) ++hw.recovery_delays;
    hw.now_us += us;
}

icm20948_status_e icm20948_set_bank(icm20948_device_t *device, uint8_t bank)
{
    if (!host_step(device, true)) return HOST_ERROR;
    assert(bank == 0 || bank == 3);
    hw.bank = bank;
    device->_last_bank = bank;
    return ICM_20948_STAT_OK;
}

static uint8_t master_status(void)
{
    ++hw.status_reads;
    /* Time advances with hardware polling, independently of how often the
     * implementation checks its per-byte and whole-acquisition deadlines. */
    hw.now_us += hw.status_step_us;
    /* DONE is latched across starts until read. A stale DONE must be drained
     * before starting; otherwise DI still belongs to the previous transaction. */
    uint8_t status = hw.status;
    hw.status = 0;
    if (status || !hw.active) return status;
    /* Hardware reproduction: continuous master mode leaves SLV4 EN asserted
     * with MST_STATUS=0, regardless of elapsed time or a master-only reset.
     * Accept both API modes below so a wrong production choice stalls here,
     * rather than failing merely because the fake asserts a chosen constant. */
    if (hw.master_mode != SAMPLE_MODE_CYCLED) return 0;
    if (++hw.pending_polls < hw.completion_polls) return 0;
    unsigned delay = hw.transactions == 1 && hw.first_completion_delay_us ?
        hw.first_completion_delay_us : hw.completion_delay_us;
    if (hw.now_us - hw.transaction_started_at_us < delay) return 0;
    unsigned fault = hw.transactions == hw.fault_transaction ? hw.fault : 0;
    if (hw.resets_needed) fault = NACK; /* CTRL=0 cannot repair a stuck master. */
    if (hw.persistent_fault) fault = hw.persistent_fault;
    if (fault == TIMEOUT) return 0;
    if (fault) return (uint8_t)(DONE | fault); /* Error wins over DONE. */
    transaction_t *txn = &hw.trace[hw.transactions - 1];
    if (txn->read) {
        /* An interrupted data read must release ST2 before consulting ST1. */
        if (txn->reg == AK09916_REG_ST1) assert(!hw.locked);
        hw.din = hw.ak[txn->reg];
        if (hw.bad_mode_readback && txn->reg == AK09916_REG_CNTL2)
            hw.din ^= 1;
        txn->value = hw.din;
        if (txn->reg == AK09916_REG_HXL) {
            hw.locked = true;
            hw.ak[AK09916_REG_ST1] &= (uint8_t)~1U;
        }
        if (txn->reg == AK09916_REG_ST2) hw.locked = false;
    } else {
        hw.ak[txn->reg] = txn->value;
        if (txn->reg == AK09916_REG_CNTL3) {
            assert(txn->value == 1);
            hw.reset_at_us = hw.now_us;
            hw.ak[AK09916_REG_CNTL2] = 0;
            hw.locked = false;
        }
        if (txn->reg == AK09916_REG_CNTL2)
            assert(hw.now_us - hw.reset_at_us >= 100);
    }
    txn->completed = true;
    hw.active = false;
    return DONE;
}

icm20948_status_e icm20948_execute_r(icm20948_device_t *device, uint8_t reg,
                                    uint8_t *data, uint32_t len)
{
    if (!host_step(device, true)) return HOST_ERROR;
    ++hw.reg_reads;
    assert(data && len == 1 && device->_last_bank == hw.bank);
    if (hw.bank == 0 && reg == AGB0_REG_I2C_MST_STATUS) {
        *data = master_status();
    } else if (hw.bank == 3 && reg == AGB3_REG_I2C_PERIPH4_CTRL) {
        if (hw.master_reset_calls && hw.gate_required && !hw.recovery_stage)
            hw.recovery_host_calls[4] = hw.host_calls;
        ++hw.ctrl_reads;
        if (hw.fail_ctrl_reads) {
            --hw.fail_ctrl_reads;
            hw.bank_error_pending = true;
            return HOST_ERROR;
        }
        *data = hw.active ? 0x80 : 0;
        if (!(*data & 0x80)) hw.gate_required = false;
    } else {
        assert(hw.bank == 3 && reg == AGB3_REG_I2C_PERIPH4_DI);
        assert(hw.transactions && !hw.active);
        assert(hw.trace[hw.transactions - 1].completed);
        *data = hw.din;
        if (hw.reg == hw.stall_reg) hw.now_us += hw.di_stall_us;
    }
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_execute_w(icm20948_device_t *device, uint8_t reg,
                                    uint8_t *data, uint32_t len)
{
    if (!host_step(device, true)) return HOST_ERROR;
    ++hw.reg_writes;
    assert(data && hw.bank == 3 && device->_last_bank == 3);
    if (len == 2) {
        assert(reg == AGB3_REG_I2C_PERIPH4_ADDR);
        hw.addr = *data++;
        if (hw.truncate_setup) {
            hw.bank_error_pending = true;
            return HOST_ERROR; /* ADDR was accepted, REG was never written. */
        }
        ++reg;
    } else {
        assert(len == 1);
    }
    switch (reg) {
    case AGB3_REG_I2C_PERIPH4_ADDR: hw.addr = *data; break;
    case AGB3_REG_I2C_PERIPH4_REG:
        hw.reg = *data;
        if (hw.reg == hw.stall_reg) hw.now_us += hw.setup_stall_us;
        break;
    case AGB3_REG_I2C_PERIPH4_DO: hw.dout = *data; break;
    case AGB3_REG_I2C_PERIPH4_CTRL:
        if (*data == 0) {
            if (hw.master_reset_calls && hw.gate_required && !hw.recovery_stage)
                hw.recovery_host_calls[3] = hw.host_calls;
            ++hw.cancels;
            if (hw.fail_cancels) {
                --hw.fail_cancels;
                hw.bank_error_pending = true;
                hw.gate_required = true;
                return ICM_20948_STAT_ERR;
            }
            if (!hw.stop_stuck) hw.active = false;
            break;
        }
        assert(*data == 0x80 && !hw.active && !hw.gate_required);
        assert((hw.addr & 0x7f) == MAG_AK09916_I2C_ADDR);
        assert(hw.transactions < TRACE_CAPACITY);
        hw.trace[hw.transactions++] = (transaction_t){
            .reg = hw.reg, .value = hw.dout, .read = (hw.addr & 0x80) != 0,
        };
        hw.active = true;
        hw.pending_polls = 0;
        hw.transaction_started_at_us = hw.now_us;
        if (hw.transactions == hw.fail_start_transaction) {
            /* Hardware accepted EN, but the host reports a transfer error. */
            hw.bank_error_pending = true;
            hw.gate_required = true;
            return HOST_ERROR;
        }
        break;
    default: assert(!"Unexpected host write");
    }
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_i2c_master_enable(icm20948_device_t *device, bool enable)
{
    if (!host_step(device, false)) return HOST_ERROR;
    if (!enable) {
        assert(hw.master_enabled && hw.recovery_stage == 0);
        hw.recovery_host_calls[0] = hw.host_calls;
        ++hw.master_disable_calls;
        if (hw.fail_recovery_api == 1) return hw.recovery_api_error;
        hw.master_enabled = false;
        hw.recovery_stage = 1;
        return ICM_20948_STAT_OK;
    }
    if (hw.recovery_stage) {
        assert(hw.recovery_stage == 2);
        hw.recovery_host_calls[2] = hw.host_calls;
        assert(hw.now_us - hw.master_reset_at_us >= 10000);
        if (hw.fail_recovery_api == 3) return hw.recovery_api_error;
        hw.recovery_stage = 0;
    }
    hw.master_enabled = true;
    ++hw.master_calls;
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_i2c_master_reset(icm20948_device_t *device)
{
    if (!host_step(device, false)) return HOST_ERROR;
    assert(!hw.master_enabled && hw.recovery_stage == 1);
    hw.recovery_host_calls[1] = hw.host_calls;
    ++hw.master_reset_calls;
    if (hw.fail_recovery_api == 2) return hw.recovery_api_error;
    hw.recovery_stage = 2;
    hw.master_reset_at_us = hw.now_us;
    if (hw.resets_needed) --hw.resets_needed;
    hw.active = false;
    hw.status = 0;
    /* Re-enabling is insufficient: CTRL readback must confirm quiescence. */
    hw.gate_required = true;
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_i2c_controller_configure_peripheral(
    icm20948_device_t *device, uint8_t peripheral, uint8_t addr, uint8_t reg,
    uint8_t len, bool read, bool enable, bool data_only, bool grp, bool swap,
    uint8_t data_out)
{
    if (!host_step(device, false)) return HOST_ERROR;
    assert(peripheral < 2 && addr == MAG_AK09916_I2C_ADDR && !enable);
    assert(hw.master_calls > hw.peripheral_calls[peripheral]);
    ++hw.peripheral_calls[peripheral];
    (void)reg; (void)len; (void)read; (void)data_only;
    (void)grp; (void)swap; (void)data_out;
    return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *device,
    icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode)
{
    if (!host_step(device, false)) return HOST_ERROR;
    assert(sensors == ICM_20948_INTERNAL_MST);
    assert(mode == SAMPLE_MODE_CYCLED || mode == SAMPLE_MODE_CONTINUOUS);
    assert(hw.master_enabled);
    assert(hw.peripheral_calls[0] > hw.sample_mode_calls);
    assert(hw.peripheral_calls[1] > hw.sample_mode_calls);
    hw.master_mode = mode;
    ++hw.sample_mode_calls;
    return ICM_20948_STAT_OK;
}

static void reset_hw(void)
{
    memset(&hw, 0, sizeof(hw));
    hw.device._last_bank = 255;
    hw.now_us = (int64_t)START;
    hw.status_step_us = 250;
    hw.completion_polls = 3;
    /* Reset LP_CONFIG=0x40; standalone polling tests start in that mode. */
    hw.master_mode = SAMPLE_MODE_CYCLED;
    hw.ak[AK09916_REG_WIA1] = 0x48;
    hw.ak[AK09916_REG_WIA2] = 0x09;
}

static void expect_transaction(unsigned index, uint8_t reg, bool read, uint8_t value)
{
    assert(index < hw.transactions);
    const transaction_t *txn = &hw.trace[index];
    assert(txn->reg == reg && txn->read == read && txn->completed);
    assert(txn->value == value);
}

static void expect_clear(const imu_magnetometer_t *mag)
{
    assert(!mag->valid && !mag->overflow && !mag->needs_release);
    assert(mag->x == 0 && mag->y == 0 && mag->z == 0);
    assert(mag->updated_at_us == 0 && mag->next_poll_us == 0);
}

static imu_magnetometer_t dirty_cache(void)
{
    return (imu_magnetometer_t){ .x = 123, .y = -456, .z = 789,
        .valid = true, .overflow = true, .needs_release = true,
        .updated_at_us = START, .next_poll_us = START + 10000 };
}

/* Check all unrelated flags, payload fields, and input immutability each time. */
static void expect_apply(const imu_magnetometer_t *mag, uint64_t now, unsigned flags)
{
    imu_magnetometer_t before;
    memcpy(&before, mag, sizeof(before));
    for (unsigned initial = 0; initial <= UINT8_MAX; ++initial) {
        imu_raw_sample_t sample, expected;
        memset(&sample, 0xa5, sizeof(sample));
        sample.flags = (uint8_t)initial;
        memcpy(&expected, &sample, sizeof(expected));
        expected.mag_x = mag->x;
        expected.mag_y = mag->y;
        expected.mag_z = mag->z;
        expected.flags = (uint8_t)((initial & ~MAG_FLAGS) | flags);
        imu_magnetometer_apply(mag, &sample, now);
        assert(memcmp(&sample, &expected, sizeof(sample)) == 0);
        assert(memcmp(mag, &before, sizeof(before)) == 0);
    }
}

static void load_axes(int16_t x, int16_t y, int16_t z, uint8_t st2)
{
    const int16_t axes[] = { x, y, z };
    for (unsigned i = 0; i < 3; ++i) {
        hw.ak[AK09916_REG_HXL + 2 * i] = (uint8_t)(uint16_t)axes[i];
        hw.ak[AK09916_REG_HXL + 2 * i + 1] = (uint8_t)((uint16_t)axes[i] >> 8);
    }
    hw.ak[AK09916_REG_ST2] = st2;
    hw.ak[AK09916_REG_ST1] = 1;
}

static void expect_ready_trace(unsigned offset)
{
    expect_transaction(offset, AK09916_REG_ST1, true, 1);
    for (unsigned i = 0; i < 6; ++i)
        expect_transaction(offset + 1 + i, AK09916_REG_HXL + i, true,
                           hw.ak[AK09916_REG_HXL + i]);
    expect_transaction(offset + READY_TRANSACTIONS - 1, AK09916_REG_ST2, true,
                       hw.ak[AK09916_REG_ST2]);
}

static void test_configure(void)
{
    reset_hw();
    imu_magnetometer_t mag = dirty_cache();
    assert(imu_magnetometer_configure(&hw.device, &mag, false) == ICM_20948_STAT_OK);
    expect_clear(&mag);
    assert(!mag.needs_recovery && hw.ctrl_reads == 1 && hw.cancels == 1);
    assert(hw.transactions == 5 && hw.delays == 1);
    assert(hw.master_calls == 1 && hw.sample_mode_calls == 1);
    assert(hw.peripheral_calls[0] == 1 && hw.peripheral_calls[1] == 1);
    expect_transaction(0, AK09916_REG_WIA1, true, 0x48);
    expect_transaction(1, AK09916_REG_WIA2, true, 0x09);
    expect_transaction(2, AK09916_REG_CNTL3, false, 1);
    expect_transaction(3, AK09916_REG_CNTL2, false, AK09916_MODE_CONT_100_HZ);
    expect_transaction(4, AK09916_REG_CNTL2, true, AK09916_MODE_CONT_100_HZ);
    /* Reconfigure an already-used reader, including outstanding release state. */
    mag = dirty_cache();
    assert(imu_magnetometer_configure(&hw.device, &mag, false) == ICM_20948_STAT_OK);
    expect_clear(&mag);
    assert(!mag.needs_recovery && hw.ctrl_reads == 2 && hw.cancels == 2);
    assert(hw.transactions == 10 && hw.delays == 2);
    assert(hw.master_calls == 2 && hw.sample_mode_calls == 2);
    assert(hw.peripheral_calls[0] == 2 && hw.peripheral_calls[1] == 2);
    expect_apply(&mag, START, 0);
    for (unsigned bad = 0; bad < 3; ++bad) {
        reset_hw();
        if (bad < 2) hw.ak[AK09916_REG_WIA1 + bad] ^= 1;
        else hw.bad_mode_readback = true;
        mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, false) ==
               (bad < 2 ? ICM_20948_STAT_WRONG_ID : ICM_20948_STAT_ERR));
        expect_clear(&mag);
        assert(mag.needs_recovery);
        if (bad < 2) {
            assert(hw.transactions == 6 && hw.recovery_delays == 2);
            assert(hw.master_reset_calls == 2 && hw.master_disable_calls == 2);
            assert(hw.reset_at_us == 0); /* Never reset/configure an unidentified AK. */
        } else {
            assert(hw.transactions == 5 && hw.master_reset_calls == 0);
        }
    }
    puts("PASS: configure disables SLV0/1, checks AK identity/reset/100Hz readback, clears repeated cache");
}

static void test_startup_master_recovery(void)
{
    for (unsigned dmp = 0; dmp < 2; ++dmp) {
        for (unsigned resets = 1; resets <= 2; ++resets) {
            reset_hw();
            hw.resets_needed = resets;
            hw.status = DONE; /* Stale success cannot hide boot nonresponse. */
            imu_magnetometer_t mag = dirty_cache();
            assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_OK);
            expect_clear(&mag);
            assert(!mag.needs_recovery && !hw.active && !hw.gate_required);
            assert(hw.resets_needed == 0 && hw.recovery_stage == 0 && hw.master_enabled);
            assert(hw.master_reset_calls == resets && hw.master_disable_calls == resets);
            assert(hw.recovery_delays == resets && hw.master_calls == resets + 1);
            assert(hw.ctrl_reads == resets + 1 && hw.transactions == resets + 5);
            for (unsigned i = 0; i < resets; ++i) {
                assert(hw.trace[i].read && hw.trace[i].reg == AK09916_REG_WIA1);
                assert(!hw.trace[i].completed);
            }
            expect_transaction(resets, AK09916_REG_WIA1, true, 0x48);
            expect_transaction(resets + 1, AK09916_REG_WIA2, true, 0x09);
            expect_transaction(resets + 2, AK09916_REG_CNTL3, false, 1);
            expect_transaction(resets + 3, AK09916_REG_CNTL2, false,
                               dmp ? 0 : AK09916_MODE_CONT_100_HZ);
            assert(hw.now_us - (int64_t)START >= (int64_t)resets * 10000);
            assert(hw.now_us - (int64_t)START < (int64_t)resets * 10000 + 15000);
        }
        /* Failure on WIA2 must restart the complete identity probe. */
        reset_hw();
        hw.fault_transaction = 2;
        hw.fault = NACK;
        imu_magnetometer_t mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_OK);
        assert(hw.master_reset_calls == 1 && hw.transactions == 7);
        expect_transaction(0, AK09916_REG_WIA1, true, 0x48);
        assert(!hw.trace[1].completed && hw.trace[1].reg == AK09916_REG_WIA2);
        expect_transaction(2, AK09916_REG_WIA1, true, 0x48);
        expect_transaction(3, AK09916_REG_WIA2, true, 0x09);
    }
    puts("PASS: raw/DMP boot nonresponse requires reset+10ms+re-enable+quiescence; third attempt can succeed");
}

static void test_startup_fault_bounds(void)
{
    const unsigned faults[] = { NACK, LOST_ARB, TIMEOUT };
    for (unsigned dmp = 0; dmp < 2; ++dmp) {
        for (unsigned f = 0; f < sizeof(faults) / sizeof(faults[0]); ++f) {
            reset_hw();
            hw.persistent_fault = faults[f];
            imu_magnetometer_t mag = dirty_cache();
            assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_ERR);
            expect_clear(&mag);
            assert(mag.needs_recovery && !hw.active);
            assert(hw.transactions == 3 && hw.master_reset_calls == 2);
            assert(hw.master_disable_calls == 2 && hw.master_calls == 3);
            assert(hw.recovery_delays == 2 && hw.ctrl_reads == 3);
            assert(hw.reset_at_us == 0); /* No AK writes before identity succeeds. */
            assert(hw.now_us - (int64_t)START < 180000);
            if (faults[f] == TIMEOUT)
                assert(hw.now_us - (int64_t)START >= 170000); /* 3*50ms + 2*10ms. */
            for (unsigned i = 0; i < hw.transactions; ++i)
                assert(!hw.trace[i].completed && hw.trace[i].reg == AK09916_REG_WIA1);
        }
        for (unsigned byte = 0; byte < 2; ++byte) {
            reset_hw();
            hw.ak[AK09916_REG_WIA1 + byte] ^= 1;
            imu_magnetometer_t mag = dirty_cache();
            assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_WRONG_ID);
            assert(hw.transactions == 6 && hw.master_reset_calls == 2);
            assert(hw.reset_at_us == 0 && mag.needs_recovery);
            expect_clear(&mag);
        }
        /* A proven identity does not license retrying or ignoring reset/mode failures. */
        for (unsigned transaction = 3; transaction <= 5; ++transaction) {
            reset_hw();
            hw.fault_transaction = transaction;
            hw.fault = NACK;
            imu_magnetometer_t mag = dirty_cache();
            assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_ERR);
            assert(hw.transactions == transaction && hw.master_reset_calls == 0);
            assert(hw.master_disable_calls == 0 && mag.needs_recovery);
            expect_clear(&mag);
        }
        reset_hw();
        hw.resets_needed = 1;
        hw.bad_mode_readback = true;
        imu_magnetometer_t mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_ERR);
        assert(hw.transactions == 6 && hw.master_reset_calls == 1 && mag.needs_recovery);
        expect_clear(&mag);
    }
    puts("PASS: persistent AUX faults stop at 3 probes/2 resets; wrong identity and reset/mode errors stay fatal");
}

static void test_startup_timing(void)
{
    for (unsigned dmp = 0; dmp < 2; ++dmp) {
        reset_hw();
        hw.first_completion_delay_us = 8000;
        imu_magnetometer_t mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_OK);
        assert(hw.transactions == 5 && hw.master_reset_calls == 0);
        assert(hw.now_us - (int64_t)START >= 8000);
        /* All setup bytes, including reset/mode write/readback, get startup timing. */
        reset_hw();
        hw.completion_delay_us = 8000;
        assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_OK);
        assert(hw.transactions == 5 && hw.master_reset_calls == 0);
        assert(hw.now_us - (int64_t)START >= 40000);
        assert(hw.now_us - (int64_t)START < 45000);

        load_axes(1, 2, 3, 0);
        unsigned count = hw.transactions;
        int64_t before = hw.now_us;
        assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_ERR);
        assert(hw.transactions == count + 1 && !hw.trace[count].completed);
        assert(hw.now_us - before >= 5000 && hw.now_us - before <= 5250);
        assert(!mag.valid && mag.needs_recovery && hw.master_reset_calls == 0);
    }
    reset_hw();
    hw.first_completion_delay_us = 8000;
    imu_magnetometer_t mag = {0};
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_ERR);
    assert(hw.transactions == 1 && !hw.trace[0].completed);
    assert(hw.now_us - (int64_t)START >= 5000 && hw.now_us - (int64_t)START <= 5250);
    puts("PASS: 8ms first/all startup completions succeed without reset; streaming still rejects at 5ms");
}

static void test_master_scheduling(void)
{
    for (unsigned dmp = 0; dmp < 2; ++dmp) {
        reset_hw();
        /* Configuration must restore cycled scheduling even if an earlier
         * reader left the master in the hardware-failing continuous mode. */
        hw.master_mode = SAMPLE_MODE_CONTINUOUS;
        imu_magnetometer_t mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, dmp) == ICM_20948_STAT_OK);
        assert(hw.master_mode == SAMPLE_MODE_CYCLED && hw.master_reset_calls == 0);

        /* A later mode regression must stall real reader polling, with no
         * fabricated DONE/NACK; the existing streaming deadline cancels it. */
        hw.master_mode = SAMPLE_MODE_CONTINUOUS;
        load_axes(123, -456, 789, 0);
        unsigned count = hw.transactions;
        int64_t before = hw.now_us;
        assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_ERR);
        assert(hw.transactions == count + 1 && !hw.trace[count].completed);
        assert(!mag.valid && mag.needs_recovery && !hw.active);
        assert(hw.now_us - before >= 5000 && hw.now_us - before <= 5250);

        hw.master_mode = SAMPLE_MODE_CYCLED;
        count = hw.transactions;
        assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) == ICM_20948_STAT_OK);
        expect_ready_trace(count);
        assert(mag.valid && !mag.needs_recovery && !mag.needs_release);
        assert(mag.x == 123 && mag.y == -456 && mag.z == 789);
    }
    puts("PASS: configure restores cycled master scheduling; continuous mode stalls SLV4 and cycled mode resumes");
}

static void test_recovery_host_errors(void)
{
    reset_hw();
    hw.resets_needed = 1;
    imu_magnetometer_t mag = {0};
    assert(imu_magnetometer_configure(&hw.device, &mag, false) == ICM_20948_STAT_OK);
    unsigned calls[5];
    memcpy(calls, hw.recovery_host_calls, sizeof(calls));
    for (unsigned i = 0; i < sizeof(calls) / sizeof(calls[0]); ++i) {
        assert(calls[i]);
        reset_hw();
        hw.resets_needed = 1;
        hw.fail_host = calls[i];
        mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, false) == HOST_ERROR);
        assert(hw.host_calls == calls[i]); /* No retry after a recovery API/host error. */
        assert(hw.master_disable_calls <= 1 && hw.master_reset_calls <= 1);
        assert(hw.device._last_bank == 255 && mag.needs_recovery);
        expect_clear(&mag);
    }
    for (unsigned api = 1; api <= 3; ++api) {
        reset_hw();
        hw.resets_needed = 1;
        hw.fail_recovery_api = api;
        hw.recovery_api_error = ICM_20948_STAT_ERR;
        mag = dirty_cache();
        assert(imu_magnetometer_configure(&hw.device, &mag, false) == ICM_20948_STAT_ERR);
        assert(hw.transactions == 1 && hw.master_disable_calls == 1);
        assert(hw.master_reset_calls <= 1 && hw.device._last_bank == 255);
        assert(mag.needs_recovery);
        expect_clear(&mag);
    }
    puts("PASS: disable/reset/re-enable/requiesce host errors propagate immediately; failed recovery APIs are checked");
}

static void test_dmp_configure(void)
{
    reset_hw();
    /* Enter DMP from an existing continuous raw-mode configuration. */
    hw.ak[AK09916_REG_CNTL2] = AK09916_MODE_CONT_100_HZ;
    for (unsigned iteration = 0; iteration < 2; ++iteration) {
        imu_magnetometer_t mag = dirty_cache();
        unsigned offset = hw.transactions;
        assert(imu_magnetometer_configure(&hw.device, &mag, true) == ICM_20948_STAT_OK);
        expect_clear(&mag);
        assert(!mag.needs_recovery && hw.ctrl_reads == iteration + 1);
        assert(hw.transactions == offset + 5 && hw.delays == iteration + 1);
        expect_transaction(offset, AK09916_REG_WIA1, true, 0x48);
        expect_transaction(offset + 1, AK09916_REG_WIA2, true, 0x09);
        expect_transaction(offset + 2, AK09916_REG_CNTL3, false, 1);
        expect_transaction(offset + 3, AK09916_REG_CNTL2, false, 0);
        expect_transaction(offset + 4, AK09916_REG_CNTL2, true, 0);
        assert(hw.ak[AK09916_REG_CNTL2] == 0);
        assert(hw.master_calls == iteration + 1 && hw.sample_mode_calls == iteration + 1);
        assert(hw.peripheral_calls[0] == iteration + 1);
        assert(hw.peripheral_calls[1] == iteration + 1);
        expect_apply(&mag, START, 0);
    }
    reset_hw();
    imu_magnetometer_t mag = dirty_cache();
    hw.bad_mode_readback = true;
    assert(imu_magnetometer_configure(&hw.device, &mag, true) != ICM_20948_STAT_OK);
    expect_clear(&mag);
    assert(mag.needs_recovery);
    puts("PASS: DMP configure verifies power-down mode, disables SLV0/1, clears repeated cache");
}

static void test_ready_and_throttle(void)
{
    const int16_t values[][3] = {
        { INT16_MIN, INT16_MAX, -1 }, { 0x1234, -0x2345, 0 },
    };
    for (unsigned v = 0; v < sizeof(values) / sizeof(values[0]); ++v) {
        reset_hw();
        imu_magnetometer_t mag = {0};
        load_axes(values[v][0], values[v][1], values[v][2], 0);
        assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_OK);
        assert(hw.transactions == READY_TRANSACTIONS && !hw.locked && !mag.needs_release);
        expect_ready_trace(0);
        assert(mag.valid && !mag.overflow && mag.updated_at_us == START);
        assert(mag.x == values[v][0] && mag.y == values[v][1] && mag.z == values[v][2]);
        assert(mag.next_poll_us == START + 10000);
        expect_apply(&mag, START, IMU_RAW_FLAG_MAG_VALID);
        unsigned calls = hw.host_calls;
        load_axes(7, 8, 9, 0);
        for (uint64_t t = START; t < START + 10000; t += 9999)
            assert(imu_magnetometer_poll(&hw.device, &mag, t) == ICM_20948_STAT_OK);
        assert(hw.host_calls == calls && hw.transactions == READY_TRANSACTIONS);
        assert(mag.x == values[v][0] && mag.updated_at_us == START);
        assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) == ICM_20948_STAT_OK);
        assert(hw.transactions == 2 * READY_TRANSACTIONS && mag.x == 7 && mag.y == 8 && mag.z == 9);
        assert(mag.updated_at_us == START + 10000);
        expect_ready_trace(READY_TRANSACTIONS);
    }
    puts("PASS: six axis bytes then ST2 exactly once, signed axes, 100Hz throttle at exact boundary");
}

static void test_not_ready_and_freshness(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    /* DOR alone is not DRDY. No data register may be consumed. */
    hw.ak[AK09916_REG_ST1] = 2;
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_OK);
    assert(hw.transactions == 1);
    expect_transaction(0, AK09916_REG_ST1, true, 2);
    expect_apply(&mag, START, 0);
    imu_magnetometer_update(&mag, -12, 34, -56, START);
    for (uint64_t elapsed = 10000; elapsed <= 100000; elapsed += 10000) {
        unsigned count = hw.transactions;
        assert(imu_magnetometer_poll(&hw.device, &mag, START + elapsed) == ICM_20948_STAT_OK);
        assert(hw.transactions == count + 1);
        expect_transaction(count, AK09916_REG_ST1, true, 2);
        assert(mag.valid && mag.updated_at_us == START);
        assert(mag.x == -12 && mag.y == 34 && mag.z == -56);
        expect_apply(&mag, START + elapsed, elapsed < 100000 ? IMU_RAW_FLAG_MAG_VALID : 0);
    }
    expect_apply(&mag, START + 99999, IMU_RAW_FLAG_MAG_VALID);
    expect_apply(&mag, START + 100001, 0);
    expect_apply(&mag, START - 1, 0);
    puts("PASS: noDRDY consumes only ST1, preserves cache, freshness expires at 100ms");
}

static void test_overflow_and_update(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    imu_magnetometer_update(&mag, 10, 20, 30, START);
    load_axes(-1, -2, -3, 8);
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) == ICM_20948_STAT_OK);
    assert(hw.transactions == READY_TRANSACTIONS && !mag.valid && mag.overflow && !mag.needs_release);
    assert(mag.updated_at_us == START + 10000 && !hw.locked);
    expect_apply(&mag, START + 10000, IMU_RAW_FLAG_MAG_OVERFLOW);
    imu_magnetometer_update(&mag, INT16_MIN, -1, INT16_MAX, START + 20000);
    assert(mag.valid && !mag.overflow && mag.updated_at_us == START + 20000);
    assert(mag.x == INT16_MIN && mag.y == -1 && mag.z == INT16_MAX);
    expect_apply(&mag, START + 20000, IMU_RAW_FLAG_MAG_VALID);
    imu_magnetometer_invalidate(&mag);
    assert(!mag.valid && !mag.overflow);
    expect_apply(&mag, START + 20000, 0);
    puts("PASS: overflow invalidates; update/invalidate/apply preserve unrelated sample fields and flags");
}

static void test_host_errors(void)
{
    for (unsigned configure = 0; configure < 2; ++configure) {
        reset_hw();
        imu_magnetometer_t mag = {0};
        load_axes(1, 2, 3, 0);
        assert((configure ? imu_magnetometer_configure(&hw.device, &mag, false) :
                imu_magnetometer_poll(&hw.device, &mag, START)) == ICM_20948_STAT_OK);
        unsigned calls = hw.host_calls;
        for (unsigned fail = 1; fail <= calls; ++fail) {
            reset_hw();
            hw.fail_host = fail;
            load_axes(1, 2, 3, 0);
            mag = dirty_cache();
            mag.needs_release = false;
            mag.next_poll_us = 0;
            icm20948_status_e status = configure ?
                imu_magnetometer_configure(&hw.device, &mag, false) :
                imu_magnetometer_poll(&hw.device, &mag, START);
            assert(status == HOST_ERROR && hw.host_calls >= fail);
            assert(hw.master_reset_calls == 0 && hw.master_disable_calls == 0);
            assert(!mag.valid && !mag.overflow && mag.needs_recovery);
            if (configure) expect_clear(&mag);
            if (hw.bank_error_pending) assert(hw.device._last_bank == 255);
            expect_apply(&mag, START, 0);
        }
        printf("PASS: %s propagates and invalidates all %u host/API failure points\n",
               configure ? "configure" : "poll", calls);
    }
}

static void test_bus_errors_and_stale_done(void)
{
    const unsigned faults[] = { NACK, LOST_ARB, TIMEOUT };
    for (unsigned i = 0; i < sizeof(faults) / sizeof(faults[0]); ++i) {
        for (unsigned configure = 0; configure < 2; ++configure) {
            reset_hw();
            imu_magnetometer_t mag = dirty_cache();
            mag.next_poll_us = 0;
            mag.needs_release = false;
            hw.status = DONE; /* Left over from a previous SLV4 operation. */
            hw.fault_transaction = 1;
            hw.fault = faults[i];
            int64_t before = hw.now_us;
            icm20948_status_e status = configure ?
                imu_magnetometer_configure(&hw.device, &mag, false) :
                imu_magnetometer_poll(&hw.device, &mag, START);
            assert(!hw.trace[0].completed);
            assert(!mag.valid && !mag.overflow && !hw.active);
            if (configure) {
                assert(status == ICM_20948_STAT_OK && !mag.needs_recovery);
                assert(hw.transactions == 6 && hw.master_reset_calls == 1);
                assert(hw.master_disable_calls == 1 && hw.recovery_delays == 1);
                expect_transaction(1, AK09916_REG_WIA1, true, 0x48);
                expect_transaction(2, AK09916_REG_WIA2, true, 0x09);
                assert(hw.cancels == 3 && hw.ctrl_reads == 2);
                if (faults[i] == TIMEOUT) {
                    assert(hw.now_us - before >= 60000 && hw.now_us - before < 70000);
                    assert(hw.status_reads < 230);
                }
            } else {
                assert(status != ICM_20948_STAT_OK && mag.needs_recovery);
                assert(hw.transactions == 1 && hw.cancels == 1);
                assert(hw.master_reset_calls == 0 && hw.master_disable_calls == 0);
            }
            if (!configure && faults[i] == TIMEOUT) {
                /* This deliberately times out the FIRST byte: its 5ms
                 * deadline is distinct from the 20ms acquisition budget. */
                assert(hw.now_us - before >= 5000 && hw.now_us - before <= 6000);
                assert(hw.status_reads < 30);
            }
            expect_apply(&mag, START, 0);
        }
    }
    reset_hw();
    imu_magnetometer_t mag = {0};
    hw.status = DONE;
    load_axes(-100, 200, -300, 0);
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_OK);
    assert(hw.transactions == READY_TRANSACTIONS && hw.status_reads == 25);
    /* At three completion reads per byte, the previous nine-byte sequence
     * used 36 status reads, 9 DI reads and 27 writes before bank selection. */
    assert(hw.reg_reads == 33 && hw.reg_writes == 16);
    printf("PASS: ready-frame host register transfers fell from 72 to %u before bank selection\n",
           hw.reg_reads + hw.reg_writes);
    assert(mag.valid && mag.x == -100 && mag.y == 200 && mag.z == -300);
    puts("PASS: startup recovers one-shot AUX faults; streaming faults retain 5ms bound; stale DONE is drained");
}

static void test_acquisition_budget(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    const uint64_t sample_time = 123; /* Deliberately unrelated to esp_timer time. */
    hw.status_step_us = 500;
    hw.completion_polls = 7; /* A byte with pacing still fits its 5ms limit. */
    int64_t before = hw.now_us;
    assert(imu_magnetometer_poll(&hw.device, &mag, sample_time) == ICM_20948_STAT_OK);
    assert(hw.now_us - before < 5000 && hw.transactions == 1);

    /* Eight individually successful bytes would take >20ms. A single budget
     * must span ST1 and the entire acquisition, rather than restarting per byte. */
    load_axes(123, -456, 789, 0);
    imu_magnetometer_update(&mag, 10, 20, 30, sample_time);
    unsigned count = hw.transactions;
    before = hw.now_us;
    icm20948_status_e status = imu_magnetometer_poll(&hw.device, &mag, sample_time + 10000);
    assert(status != ICM_20948_STAT_OK);
    assert(hw.now_us - before >= 20000);
    assert(hw.now_us - before <= 20000 + (int64_t)hw.status_step_us);
    assert(!mag.valid && !mag.overflow && mag.needs_release);
    assert(hw.transactions > count + 1 && hw.transactions < count + READY_TRANSACTIONS);
    expect_apply(&mag, sample_time + 10000, 0);

    /* A budget abort after HXL has the same release obligation as a bus error. */
    count = hw.transactions;
    hw.status_step_us = 250;
    hw.completion_polls = 3;
    load_axes(-321, 654, -987, 0);
    assert(imu_magnetometer_poll(&hw.device, &mag, sample_time + 30000) == ICM_20948_STAT_OK);
    expect_transaction(count, AK09916_REG_ST2, true, 0);
    expect_ready_trace(count + 1);
    assert(hw.transactions == count + 1 + READY_TRANSACTIONS && mag.valid && !mag.needs_release);
    assert(mag.x == -321 && mag.y == 654 && mag.z == -987);
    assert(mag.updated_at_us == sample_time + 30000);
    puts("PASS: 20ms acquisition budget spans individually sub-5ms bytes and releases partial frame");

    reset_hw();
    mag = (imu_magnetometer_t){.frame_timeout_us = IMU_MAG_SLOW_FRAME_TIMEOUT_US};
    hw.status_step_us = 500;
    hw.completion_polls = 7;
    load_axes(123, -456, 789, 0);
    before = hw.now_us;
    assert(imu_magnetometer_poll(&hw.device, &mag, sample_time) == ICM_20948_STAT_OK);
    assert(hw.now_us - before > 20000 && hw.now_us - before < IMU_MAG_SLOW_FRAME_TIMEOUT_US);
    assert(hw.transactions == READY_TRANSACTIONS && mag.valid && !mag.needs_release);
    puts("PASS: the slow-clock budget accepts a complete frame that takes over 20ms");
}

static void test_paced_completion(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    hw.completion_polls = 1;
    hw.completion_delay_us = 1150;
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == ICM_20948_STAT_OK);
    assert(hw.transactions == 1 && hw.trace[0].completed && hw.delays > 0);
    /* An unpaced poller would need five completion reads and one drain. */
    assert(hw.status_reads < 6);
    puts("PASS: pacing reduces status reads while waiting for hardware completion");
}

static void test_partial_release(void)
{
    /* Fail each byte from HXL through ST2, then fail the release once too. */
    for (unsigned byte = 0; byte < READY_TRANSACTIONS - 1; ++byte) {
        reset_hw();
        imu_magnetometer_t mag = {0};
        imu_magnetometer_update(&mag, 3, 4, 5, START);
        load_axes(-123, 456, -789, 0);
        hw.fault_transaction = 2 + byte;
        hw.fault = NACK;
        assert(imu_magnetometer_poll(&hw.device, &mag, START) != ICM_20948_STAT_OK);
        assert(!mag.valid && !mag.overflow && mag.needs_release);
        expect_apply(&mag, START, 0);
        unsigned calls = hw.host_calls, count = hw.transactions;
        assert(count == 2 + byte);
        assert(imu_magnetometer_poll(&hw.device, &mag, START + 9999) == ICM_20948_STAT_OK);
        assert(hw.host_calls == calls && hw.transactions == count);
        hw.fault_transaction = count + 1;
        hw.fault = LOST_ARB;
        assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) != ICM_20948_STAT_OK);
        assert(hw.transactions == count + 1 && hw.trace[count].reg == AK09916_REG_ST2);
        assert(mag.needs_release && !mag.valid);
        count = hw.transactions;
        hw.fault = 0;
        load_axes(321, -654, 987, 0);
        assert(imu_magnetometer_poll(&hw.device, &mag, START + 20000) == ICM_20948_STAT_OK);
        expect_transaction(count, AK09916_REG_ST2, true, 0);
        expect_ready_trace(count + 1);
        assert(hw.transactions == count + 1 + READY_TRANSACTIONS && !hw.locked && !mag.needs_release);
        assert(mag.valid && mag.x == 321 && mag.y == -654 && mag.z == 987);
        assert(mag.updated_at_us == START + 20000);
    }
    puts("PASS: all axis/ST2 failures release ST2 before ST1/data; failed release retries safely");
}

static void test_truncated_setup(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    imu_magnetometer_update(&mag, 1, 2, 3, START);
    load_axes(-1, -2, -3, 0);
    hw.truncate_setup = true;
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == HOST_ERROR);
    assert(hw.addr == (MAG_AK09916_I2C_ADDR | 0x80) && hw.reg == 0);
    assert(hw.transactions == 0 && !hw.active && !mag.valid && mag.needs_recovery);
    expect_apply(&mag, START, 0);

    hw.truncate_setup = false;
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) == ICM_20948_STAT_OK);
    assert(hw.ctrl_reads == 1 && hw.transactions == READY_TRANSACTIONS);
    expect_ready_trace(0);
    assert(mag.valid && mag.x == -1 && mag.y == -2 && mag.z == -3);
    puts("PASS: a truncated ADDR+REG write cannot arm SLV4 and the next poll recovers");
}

static void test_deadline_edges(void)
{
    reset_hw();
    imu_magnetometer_t mag = {0};
    imu_magnetometer_update(&mag, 1, 2, 3, START);
    hw.stall_reg = AK09916_REG_ST1;
    hw.setup_stall_us = 5000;
    assert(imu_magnetometer_poll(&hw.device, &mag, START) != ICM_20948_STAT_OK);
    assert(hw.transactions == 0 && !mag.valid && !mag.overflow);
    expect_apply(&mag, START, 0);

    reset_hw();
    mag = (imu_magnetometer_t){0};
    imu_magnetometer_update(&mag, 1, 2, 3, START);
    hw.status_step_us = 1250; /* Drain + three paced polls: DONE arrives after 5ms. */
    assert(imu_magnetometer_poll(&hw.device, &mag, START) != ICM_20948_STAT_OK);
    assert(hw.transactions == 1 && hw.trace[0].completed && !mag.valid);
    expect_apply(&mag, START, 0);

    for (unsigned whole_budget = 0; whole_budget < 2; ++whole_budget) {
        reset_hw();
        mag = (imu_magnetometer_t){0};
        imu_magnetometer_update(&mag, 1, 2, 3, START);
        load_axes(-123, 456, -789, 0);
        hw.stall_reg = AK09916_REG_ST2;
        /* All eight bytes complete, but the final DI host transfer returns late.
         * Exercise both the 5ms byte limit and the capped 20ms poll limit. */
        hw.status_step_us = whole_budget ? 500 : 250;
        hw.completion_polls = whole_budget ? 4 : 3;
        hw.di_stall_us = whole_budget ? 1250 : 5000;
        assert(imu_magnetometer_poll(&hw.device, &mag, START) != ICM_20948_STAT_OK);
        assert(hw.transactions == READY_TRANSACTIONS && hw.trace[READY_TRANSACTIONS - 1].completed && !hw.locked);
        assert(!mag.valid && !mag.overflow);
        assert(mag.updated_at_us == START && mag.x == 1 && mag.y == 2 && mag.z == 3);
        expect_apply(&mag, START, 0);
    }
    puts("PASS: expired setup cannot trigger; late DONE/final DI cannot validate frame at either deadline");
}

static void test_confirmed_recovery_gate(void)
{
    for (unsigned failure = 0; failure < 3; ++failure) {
        reset_hw();
        imu_magnetometer_t mag = dirty_cache();
        hw.active = hw.gate_required = true; /* Stale transaction at initialization. */
        hw.fail_cancels = failure == 0;
        hw.fail_ctrl_reads = failure == 1;
        hw.stop_stuck = failure == 2;
        icm20948_status_e expected = failure == 1 ? HOST_ERROR : ICM_20948_STAT_ERR;
        assert(imu_magnetometer_configure(&hw.device, &mag, false) == expected);
        expect_clear(&mag);
        assert(mag.needs_recovery && hw.transactions == 0 && hw.gate_required);
        hw.stop_stuck = false;
        assert(imu_magnetometer_configure(&hw.device, &mag, false) == ICM_20948_STAT_OK);
        expect_clear(&mag);
        assert(!mag.needs_recovery && hw.transactions == 5 && !hw.gate_required);
    }

    reset_hw();
    imu_magnetometer_t mag = {0};
    imu_magnetometer_update(&mag, 10, 20, 30, START);
    load_axes(-1, -2, -3, 0);
    hw.fail_start_transaction = 3; /* HXH starts after HXL has locked the frame. */
    hw.fail_cancels = 1; /* Best-effort cancel also fails with a DIFFERENT status. */
    assert(imu_magnetometer_poll(&hw.device, &mag, START) == HOST_ERROR);
    assert(hw.transactions == 3 && hw.active && hw.locked && hw.gate_required);
    assert(mag.needs_recovery && mag.needs_release && !mag.valid && !mag.overflow);
    expect_apply(&mag, START, 0);
    unsigned calls = hw.host_calls;
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 9999) == ICM_20948_STAT_OK);
    assert(hw.host_calls == calls && mag.needs_recovery);

    hw.fail_cancels = 1; /* Retry cannot proceed past a failed CTRL=0 write. */
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 10000) == ICM_20948_STAT_ERR);
    assert(hw.transactions == 3 && hw.ctrl_reads == 0 && hw.active);
    assert(mag.needs_recovery && mag.needs_release && !mag.valid);
    hw.fail_ctrl_reads = 1; /* A successful stop alone is insufficient. */
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 20000) == HOST_ERROR);
    assert(hw.transactions == 3 && hw.ctrl_reads == 1 && hw.gate_required);
    assert(mag.needs_recovery && mag.needs_release && !mag.valid);
    hw.active = hw.stop_stuck = true; /* Readback explicitly reports EN still set. */
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 30000) == ICM_20948_STAT_ERR);
    assert(hw.transactions == 3 && hw.ctrl_reads == 2 && hw.active);
    assert(mag.needs_recovery && mag.needs_release && !mag.valid);

    hw.stop_stuck = false;
    load_axes(123, -456, 789, 0);
    assert(imu_magnetometer_poll(&hw.device, &mag, START + 40000) == ICM_20948_STAT_OK);
    assert(hw.ctrl_reads == 3 && !hw.gate_required);
    expect_transaction(3, AK09916_REG_ST2, true, 0);
    expect_ready_trace(4);
    assert(hw.transactions == 4 + READY_TRANSACTIONS && !hw.active && !hw.locked);
    assert(!mag.needs_recovery && !mag.needs_release && mag.valid);
    assert(mag.x == 123 && mag.y == -456 && mag.z == 789);
    puts("PASS: configure/poll require confirmed CTRL EN=0; ambiguous start and failed cancellation stay gated; original error preserved");
}

int main(void)
{
    _Static_assert(IMU_MAG_FRESHNESS_US == 100000, "100ms freshness contract");
    _Static_assert(IMU_MAG_POLL_PERIOD_US == 10000, "100Hz polling contract");
    test_configure();
    test_dmp_configure();
    test_startup_master_recovery();
    test_startup_fault_bounds();
    test_startup_timing();
    test_master_scheduling();
    test_recovery_host_errors();
    test_ready_and_throttle();
    test_not_ready_and_freshness();
    test_overflow_and_update();
    test_host_errors();
    test_bus_errors_and_stale_done();
    test_partial_release();
    test_truncated_setup();
    test_paced_completion();
    test_acquisition_budget();
    test_deadline_edges();
    test_confirmed_recovery_gate();
    puts("PASS: magnetometer regression (ASan + UBSan)");
    return 0;
}
