#!/usr/bin/env python3
"""Bounded sender interleavings using verbatim production sender functions.

SDK/event scheduling is shimmed; the real queue, codec, host callback and sender
loop execute unchanged. Unrelated GAP/security/init functions are omitted from
the temporary translation unit. No generated files escape TemporaryDirectory.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADERS = {
    "esp_err.h": "#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NO_MEM 1\n",
    "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_system.h": "#pragma once\n#include <stdint.h>\nuint32_t esp_get_free_heap_size(void);\n",
    "sdkconfig.h": "#pragma once\n#define CONFIG_AUX_ADC_ENABLED 1\n#define CONFIG_AUX_ADC_CHANNEL_COUNT 8\n#define CONFIG_IMU_RAW_ENABLED 1\n",
    "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
""",
    "freertos/semphr.h": """#pragma once
#include "freertos/FreeRTOS.h"
typedef struct native_sem *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
""",
}


def between(source, start, end):
    return source[source.index(start):source.index(end)]


def main():
    manager = (ROOT / "main/ble/ble_manager.c").read_text()
    sender = (
        between(manager, "enum { BATCH_FLUSH_MS", "static const char *hci_reason_name")
        + between(manager, "static bool stream_is_current(", "static void check_pairing_timeout(")
        + between(manager, "static uint16_t stream_handle(", "esp_err_t ble_manager_init(")
    )
    with tempfile.TemporaryDirectory(prefix="glove-sender-") as temp:
        directory = Path(temp)
        for name, contents in HEADERS.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
        (directory / "sender_under_test.inc").write_text(sender)
        binary = directory / "sender_regression"
        subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            f"-I{directory}", f"-I{ROOT / 'main'}",
            str(ROOT / "tests/native/sender_regression.c"),
            str(ROOT / "main/ble/ble_sample_buffer.c"),
            str(ROOT / "main/ble/ble_batch_packets.c"),
            "-o", str(binary),
        ], check=True, timeout=30)
        result = subprocess.run([str(binary)], check=False, timeout=10)
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
