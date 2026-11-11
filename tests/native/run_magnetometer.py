#!/usr/bin/env python3
"""Compile the real magnetometer reader against managed headers; run ASan/UBSan."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    managed = root / "managed_components" / "cybergear-robotics__icm20948"
    if not (managed / "icm20948.h").is_file():
        raise SystemExit(f"Missing actual managed driver header: {managed / 'icm20948.h'}")
    headers = {
        "sdkconfig.h": "#pragma once\n#define CONFIG_AUX_ADC_ENABLED 0\n",
        "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
        "esp_rom_sys.h": "#pragma once\n#include <stdint.h>\nvoid esp_rom_delay_us(uint32_t us);\n",
    }
    with tempfile.TemporaryDirectory(prefix="glove-magnetometer-") as temp:
        directory = Path(temp)
        for name, contents in headers.items():
            (directory / name).write_text(contents)
        binary = directory / "magnetometer_regression"
        print(f"Magnetometer: actual managed headers at {managed.resolve()}", flush=True)
        subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            "-fno-omit-frame-pointer", f"-I{directory}", f"-I{managed}",
            f"-I{root / 'main'}", str(root / "main/imu/magnetometer_reader.c"),
            str(Path(__file__).with_name("magnetometer_regression.c")),
            "-o", str(binary),
        ], check=True, timeout=60)
        env = os.environ.copy()
        # Append so inherited options cannot silently make sanitizer errors pass.
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":halt_on_error=1:abort_on_error=1"
        env["UBSAN_OPTIONS"] = env.get("UBSAN_OPTIONS", "") + ":halt_on_error=1:print_stacktrace=1"
        subprocess.run([str(binary)], check=True, env=env, timeout=30)


if __name__ == "__main__":
    main()
