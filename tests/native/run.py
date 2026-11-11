#!/usr/bin/env python3
"""Run production packet/state/ADC code with sanitizer-backed host SDK shims."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
headers = {
    "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_INVALID_ARG 2
#define ESP_ERR_INVALID_STATE 3
#define ESP_FAIL 4
""",
    "esp_log.h": """#pragma once
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
""",
    "esp_timer.h": "#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_system.h": "#include <stdint.h>\nuint32_t esp_get_free_heap_size(void);\nuint32_t esp_get_minimum_free_heap_size(void);\n",
    "esp_heap_caps.h": "",
    "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
#define portMAX_DELAY UINT32_MAX
#define pdPASS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
""",
    "freertos/semphr.h": """#pragma once
#include <pthread.h>
#include <stdlib.h>
typedef pthread_mutex_t *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    SemaphoreHandle_t m = malloc(sizeof(*m));
    if (m) pthread_mutex_init(m, NULL);
    return m;
}
#define xSemaphoreTake(m, timeout) pthread_mutex_lock(m)
#define xSemaphoreGive(m) pthread_mutex_unlock(m)
""",
    "freertos/task.h": """#pragma once
#include "freertos/FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last, TickType_t period);
int xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
                void *arg, unsigned priority, void *handle);
""",
    "driver/i2c_master.h": "#pragma once\ntypedef void *i2c_master_bus_handle_t;\n",
    "esp_adc/adc_oneshot.h": """#pragma once
#include "esp_err.h"
typedef void *adc_oneshot_unit_handle_t;
typedef int adc_channel_t;
typedef int adc_unit_t;
typedef int adc_atten_t;
typedef int adc_bitwidth_t;
enum { ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_12 };
typedef struct { adc_bitwidth_t bitwidth; adc_atten_t atten; } adc_oneshot_chan_cfg_t;
esp_err_t adc_oneshot_io_to_channel(int gpio, adc_unit_t *unit, adc_channel_t *channel);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t unit,
                                   adc_channel_t channel, const adc_oneshot_chan_cfg_t *cfg);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t unit, adc_channel_t channel, int *raw);
""",
}


def config(target, count, dummy=False, bad_gpio=None):
    enabled = count > 0
    values = {
        "CONFIG_IMU_RAW_RATE_HZ": 100,
        "CONFIG_BATTERY_STATUS_PERIOD_MS": 1000,
        "CONFIG_BLE_COMPACT_PACKET_FALLBACK": 1,
        "TEST_EXPECTED_CHANNEL_COUNT": count,
        "CONFIG_IDF_TARGET_ESP32" + target.upper(): 1,
        "CONFIG_TARGET_BOARD_XIAO_ESP32_" + target.upper(): 1,
        "CONFIG_XIAO_C3_I2C_SCL_GPIO": 7,
        "CONFIG_XIAO_C3_I2C_SDA_GPIO": 6,
        "CONFIG_XIAO_C3_LED_GPIO": 21,
        "CONFIG_XIAO_C3_BATTERY_ADC_GPIO": 2,
        "CONFIG_XIAO_C3_BUTTON_GPIO": 9,
        "CONFIG_XIAO_C3_POWER_HOLD_GPIO": 5,
        "CONFIG_XIAO_S3_I2C_SCL_GPIO": 6,
        "CONFIG_XIAO_S3_I2C_SDA_GPIO": 5,
        "CONFIG_XIAO_S3_LED_GPIO": 43,
        "CONFIG_XIAO_S3_BATTERY_ADC_GPIO": 1,
        "CONFIG_XIAO_S3_BUTTON_GPIO": 0,
        "CONFIG_XIAO_S3_POWER_HOLD_GPIO": 4,
    }
    if enabled:
        values.update(CONFIG_AUX_ADC_ENABLED=1, CONFIG_AUX_ADC_CHANNEL_COUNT=count,
                      CONFIG_ADC_RATE_HZ=100, CONFIG_ADC_WIDTH_BITS=12)
        if dummy:
            values["CONFIG_ADC_DUMMY"] = 1
        else:
            values.update(CONFIG_ADC_AVERAGE_SAMPLES=2, CONFIG_ADC_ATTEN_12DB=1)
            pins = [0, 1, 3] if target == "c3" else [2, 3, 7, 8, 9, 10, 11, 12]
            for i in range(count):
                values[f"CONFIG_AUX_ADC_GPIO_{i}"] = pins[i]
            if bad_gpio is not None:
                values[f"CONFIG_AUX_ADC_GPIO_{count - 1}"] = bad_gpio
    # Inactive GPIO macros deliberately remain undefined. This catches accidental
    # references to slots that menuconfig hides, including real-only dummy options.
    return "".join(f"#define {key} {value}\n" for key, value in values.items())


def compile_binary(directory, name, sources, sdkconfig):
    (directory / "sdkconfig.h").write_text(sdkconfig)
    binary = directory / name
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra", "-Wtype-limits", "-Werror",
        # Disabled features may leave static functions in production code.
        "-Wno-unused-function", "-Wno-unused-variable",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-pthread",
        f"-I{directory}", f"-I{root / 'main'}",
        *map(str, sources), "-o", str(binary),
    ], check=True)
    return binary


def run(binary, *args):
    subprocess.run([str(binary), *args], check=True)


with tempfile.TemporaryDirectory(prefix="glove-regression-") as temp:
    directory = Path(temp)
    for name, contents in headers.items():
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    packet_sources = [root / "main/system_state.c", root / "main/ble/ble_packets.c",
                      Path(__file__).with_name("regression.c")]
    adc_sources = [root / "main/system_state.c", root / "main/adc/adc_manager.c",
                   Path(__file__).with_name("adc_regression.c")]
    for target, count in [("c3", 3), ("c3", 1), ("s3", 8), ("c3", 0)]:
        name = f"{target}_{count}"
        print(f"Packet/state: {name}", flush=True)
        run(compile_binary(directory, "packet_" + name, packet_sources, config(target, count)))
        modes = [False, True] if count else [False]
        for dummy in modes:
            name = f"{target}_{count}_{'dummy' if dummy else 'real'}"
            print(f"ADC manager: {name}", flush=True)
            binary = compile_binary(directory, name, adc_sources, config(target, count, dummy))
            run(binary, "sample")
            if count:
                run(binary, "task_fail")
            if count and not dummy:
                for scenario in ["map_fail", "unit_fail", "config_fail",
                                 "read_fail_first", "read_fail_last"]:
                    run(binary, scenario)
    # Fail the final mapping slot so validation must finish before allocating or
    # configuring ANY channel. GPIO0 duplicates slot0; others are reserved/invalid.
    for pin in [0, 2, 5, 6, 7, 9, 21, -1, 48]:
        print(f"ADC manager: invalid final GPIO {pin}", flush=True)
        binary = compile_binary(directory, "invalid_gpio", adc_sources,
                                config("c3", 3, bad_gpio=pin))
        run(binary, "invalid_gpio")
