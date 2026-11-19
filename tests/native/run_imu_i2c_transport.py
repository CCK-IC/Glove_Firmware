#!/usr/bin/env python3
"""Exercise the IMU transport and the managed driver with an ESP-IDF I2C shim."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


root = Path(__file__).resolve().parents[2]
managed = root / "managed_components/cybergear-robotics__icm20948"
headers = {
    "driver/i2c_master.h": """#pragma once
#include <stddef.h>
#include <stdint.h>
typedef void *i2c_master_dev_handle_t;
typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT,
       ESP_ERR_INVALID_RESPONSE };
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device, const uint8_t *data,
                              size_t length, int timeout_ms);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
                                      const uint8_t *write_data, size_t write_length,
                                      uint8_t *read_data, size_t read_length, int timeout_ms);
""",
}

with tempfile.TemporaryDirectory(prefix="glove-imu-transport-") as temp:
    directory = Path(temp)
    for name, contents in headers.items():
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    for use_dmp in (0, 1):
        (directory / "sdkconfig.h").write_text(
            f"#define CONFIG_IMU_ENABLED 1\n#define CONFIG_ICM_20948_USE_DMP {use_dmp}\n"
        )
        binary = directory / "transport_regression"
        subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")), "-std=gnu11", "-Wall", "-Wextra",
            "-Wno-unused-parameter", "-Wno-unused-variable",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-g",
            f"-I{directory}", f"-I{managed}", f"-I{root / 'main'}",
            str(Path(__file__).with_name("imu_i2c_transport_regression.c")),
            str(root / "main/imu/imu_i2c_transport.c"), str(managed / "icm20948.c"),
            "-o", str(binary),
        ], check=True, timeout=60)
        subprocess.run([str(binary)], check=True, timeout=15)
