#!/usr/bin/env python3
"""Compile the actual sample queue with pthread SDK shims and ASan/UBSan.

All generated headers and executables live in a temporary directory. Production
headers (including system_state_t and its counter offset) are used unchanged.
Run from any directory; CC may specify a compiler and additional arguments.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADERS = {
    "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 0x101
""",
    "sdkconfig.h": """#pragma once
#define CONFIG_AUX_ADC_ENABLED 1
#define CONFIG_AUX_ADC_CHANNEL_COUNT 8
#define CONFIG_IMU_DMP_DUMMY 1
#define CONFIG_IMU_RAW_DUMMY 1
""",
    "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
""",
    "freertos/semphr.h": """#pragma once
#include "freertos/FreeRTOS.h"
#include <pthread.h>
typedef pthread_mutex_t *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex);
""",
}


def main():
    with tempfile.TemporaryDirectory(prefix="glove-sample-buffer-") as temp:
        directory = Path(temp)
        for name, contents in HEADERS.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
        binary = directory / "sample_buffer_regression"
        command = [
            *shlex.split(os.environ.get("CC", "cc")),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread",
            "-g", "-O1", "-fsanitize=address,undefined",
            "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
            f"-I{directory}", f"-I{ROOT / 'main'}",
            str(ROOT / "main/ble/ble_sample_buffer.c"),
            str(ROOT / "tests/native/sample_buffer_regression.c"),
            "-o", str(binary),
        ]
        print("Building production ble_sample_buffer.c with ASan/UBSan + pthreads", flush=True)
        subprocess.run(command, check=True, timeout=60)
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":halt_on_error=1:abort_on_error=1"
        env["UBSAN_OPTIONS"] = env.get("UBSAN_OPTIONS", "") + ":halt_on_error=1:print_stacktrace=1"
        subprocess.run([str(binary)], check=True, env=env, timeout=60)
        print("PASS: production sample buffer, ASan/UBSan, pthread mutex shims", flush=True)


if __name__ == "__main__":
    main()
