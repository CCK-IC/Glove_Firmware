#!/usr/bin/env python3
"""Exercise actual imu_manager DMP integration with managed types and ASan/UBSan.

All FIFO error recovery, reset gating, and terminal deadline regressions run
unconditionally alongside normal Compass acquisition and raw publication.
"""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
if len(sys.argv) != 1:
    raise SystemExit('Usage: python3 tests/native/run_dmp_magnetometer.py')
headers = {
    'sdkconfig.h': '''#pragma once
#define CONFIG_IMU_ENABLED 1
#define CONFIG_IMU_DMP_ENABLED 1
#define CONFIG_IMU_RAW_ENABLED 1
#define CONFIG_ICM_20948_USE_DMP 1
#define CONFIG_IMU_DMP_RATE_HZ 100
#define CONFIG_IMU_RAW_RATE_HZ 100
#define CONFIG_IMU_FIFO_POLL_RATE_HZ 200
#define CONFIG_IMU_ERROR_REINIT_COUNT 5
#define CONFIG_IMU_I2C_ADDRESS 0x68
#define CONFIG_IMU_I2C_RETRIES 3
#define CONFIG_BOARD_I2C_FREQUENCY_HZ 400000
#define CONFIG_AUX_ADC_ENABLED 0
''',
    'esp_err.h': '''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_NOT_FOUND 2
#define ESP_ERR_INVALID_STATE 3
#define ESP_ERR_NOT_SUPPORTED 4
#define ESP_ERR_TIMEOUT 5
#define ESP_ERR_INVALID_RESPONSE 6
const char *esp_err_to_name(esp_err_t error);
''',
    'esp_log.h': '''#pragma once
static inline void test_log(const char *tag, const char *format, ...) {
    (void)tag; (void)format;
}
#define ESP_LOGE(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGI(...) test_log(__VA_ARGS__)
''',
    'esp_timer.h': '#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
    'esp_rom_sys.h': '#pragma once\n#include <stdint.h>\nvoid esp_rom_delay_us(uint32_t us);\n',
    'freertos/FreeRTOS.h': '''#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
#define portMAX_DELAY UINT32_MAX
#define pdPASS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
''',
    'freertos/semphr.h': '''#pragma once
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static inline int xSemaphoreTake(SemaphoreHandle_t m, TickType_t t) { (void)m; (void)t; return 1; }
static inline int xSemaphoreGive(SemaphoreHandle_t m) { (void)m; return 1; }
static inline void vSemaphoreDelete(SemaphoreHandle_t m) { (void)m; }
''',
    'freertos/task.h': '''#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last, TickType_t ticks);
void vTaskDelete(TaskHandle_t task);
int xTaskCreate(void (*task)(void *), const char *name, unsigned stack, void *arg, unsigned priority, TaskHandle_t *handle);
''',
    'driver/i2c_master.h': '''#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;
typedef struct { int dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz; } i2c_device_config_t;
#define I2C_ADDR_BIT_LEN_7 0
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *config, i2c_master_dev_handle_t *device);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address, int timeout_ms);
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus);
''',
    'esp_adc/adc_oneshot.h': '#pragma once\ntypedef int adc_unit_t;\ntypedef void *adc_oneshot_unit_handle_t;\n',
}
with tempfile.TemporaryDirectory(prefix='glove-dmp-magnetometer-') as temp:
    directory = Path(temp)
    for name, contents in headers.items():
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    binary = directory / 'dmp_magnetometer_regression'
    managed = root / 'managed_components/cybergear-robotics__icm20948'
    strip = ['-Wl,-dead_strip'] if sys.platform == 'darwin' else ['-Wl,--gc-sections']
    subprocess.run([
        *shlex.split(os.environ.get('CC', 'cc')), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
        '-Wno-unused-function', '-Wno-unused-variable', '-g', '-O1',
        '-ffunction-sections', '-fdata-sections', '-fsanitize=address,undefined',
        '-fno-sanitize-recover=all', '-fno-omit-frame-pointer',
        f'-I{directory}', f'-I{managed}', f'-I{root / "main"}',
        str(Path(__file__).with_name('dmp_magnetometer_regression.c')),
        str(root / 'main/imu/magnetometer_reader.c'), *strip, '-lm', '-o', str(binary),
    ], check=True, timeout=60)
    env = os.environ.copy()
    env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '') + ':halt_on_error=1:abort_on_error=1'
    env['UBSAN_OPTIONS'] = env.get('UBSAN_OPTIONS', '') + ':halt_on_error=1:print_stacktrace=1'
    result = subprocess.run([str(binary)], env=env, timeout=30)
    raise SystemExit(result.returncode)
