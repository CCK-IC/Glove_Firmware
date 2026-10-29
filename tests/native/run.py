#!/usr/bin/env python3
"""Run the actual packet encoder and state implementation with host SDK shims."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
headers = {
    "sdkconfig.h": """
#define CONFIG_IMU_RAW_RATE_HZ 100
#define CONFIG_ADC_RATE_HZ 100
#define CONFIG_BATTERY_STATUS_PERIOD_MS 1000
#define CONFIG_BLE_COMPACT_PACKET_FALLBACK 1
""",
    "esp_err.h": "#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NO_MEM 1\n",
    "esp_log.h": "#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n",
    "esp_timer.h": "#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_system.h": "#include <stdint.h>\nuint32_t esp_get_free_heap_size(void);\nuint32_t esp_get_minimum_free_heap_size(void);\n",
    "esp_heap_caps.h": "",
    "freertos/FreeRTOS.h": "#pragma once\n#define portMAX_DELAY 0\n",
    "freertos/semphr.h": """
#pragma once
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
}
with tempfile.TemporaryDirectory(prefix="glove-regression-") as temp:
    directory = Path(temp)
    for name, contents in headers.items():
        target = directory / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(contents)
    binary = directory / "regression"
    # strlcpy is supplied by macOS libc; the firmware is built on this host.
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=address,undefined", "-pthread", f"-I{directory}",
        f"-I{root / 'main'}", str(root / "main/system_state.c"),
        str(root / "main/ble/ble_packets.c"), str(Path(__file__).with_name("regression.c")),
        "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
