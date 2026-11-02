#!/usr/bin/env python3
"""Check feature branches without changing the configured build or sdkconfig."""
import json
import re
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
entries = json.loads((root / "build/compile_commands.json").read_text())
base = (root / "build/config/sdkconfig.h").read_text()
variants = {'current': {},
 'raw': {'CONFIG_IMU_ENABLED': 1,
         'CONFIG_IMU_RAW_ENABLED': 1,
         'CONFIG_IMU_RAW_DUMMY': None,
         'CONFIG_IMU_DMP_DUMMY': None,
         'CONFIG_IMU_I2C_ADDRESS': '0x68',
         'CONFIG_IMU_I2C_RETRIES': 3},
 'dmp': {'CONFIG_IMU_ENABLED': 1,
         'CONFIG_IMU_DMP_ENABLED': 1,
         'CONFIG_IMU_DMP_RATE_HZ': 100,
         'CONFIG_IMU_DMP_DUMMY': None,
         'CONFIG_IMU_RAW_DUMMY': None,
         'CONFIG_IMU_RAW_ENABLED': 1,
         'CONFIG_IMU_I2C_ADDRESS': '0x68',
         'CONFIG_IMU_I2C_RETRIES': 3,
         'CONFIG_IMU_ERROR_REINIT_COUNT': 5,
         'CONFIG_ICM_20948_USE_DMP': 1,
         'CONFIG_IMU_SYNTHETIC_FALLBACK': 1},
 'adc': {'CONFIG_ADC_DUMMY': None,
         'CONFIG_ADC_AVERAGE_SAMPLES': 2,
         'CONFIG_AUX_ADC_GPIO_0': 1,
         'CONFIG_AUX_ADC_GPIO_1': 2,
         'CONFIG_AUX_ADC_GPIO_2': 3,
         'CONFIG_AUX_ADC_GPIO_3': 4,
         'CONFIG_AUX_ADC_GPIO_4': 5,
         'CONFIG_AUX_ADC_GPIO_5': 6,
         'CONFIG_AUX_ADC_GPIO_6': 7,
         'CONFIG_AUX_ADC_GPIO_7': 8},
 'disabled': {'CONFIG_AUX_ADC_ENABLED': None,
              'CONFIG_ADC_DUMMY': None,
              'CONFIG_IMU_RAW_DUMMY': None,
              'CONFIG_IMU_DMP_DUMMY': None,
              'CONFIG_IMU_RAW_RATE_HZ': None,
              'CONFIG_ADC_RATE_HZ': None,
              'CONFIG_IMU_SLOW_RATE_HZ': None}}

failed = False
for name, changes in variants.items():
    if len(sys.argv) > 1 and name not in sys.argv[1:]:
        continue
    with tempfile.TemporaryDirectory(prefix="glove-config-") as directory:
        header = base
        for key, value in changes.items():
            header = re.sub(r"^#define " + key + r"\b.*\n", "", header, flags=re.M)
            if value is not None:
                header += f"\n#define {key} {value}\n"
        Path(directory, "sdkconfig.h").write_text(header)
        errors = []
        for entry in entries:
            if not Path(entry["file"]).is_relative_to(root / "main"):
                continue
            args = shlex.split(entry["command"])
            command = [args[0], "-I" + directory]
            index = 1
            while index < len(args):
                if args[index] in ("-o", "-MF", "-MT", "-MQ"):
                    index += 2
                    continue
                if args[index] in ("-c", "-MD", "-MMD", "-MP"):
                    index += 1
                    continue
                command.append(args[index])
                index += 1
            result = subprocess.run(
                command + ["-fsyntax-only"], cwd=entry["directory"],
                capture_output=True, text=True,
            )
            if result.returncode:
                errors.append(Path(entry["file"]).name + ":\n" + result.stderr)
        print(name + ": " + ("FAIL" if errors else "PASS"))
        if errors:
            print("\n".join(errors))
            failed = True
sys.exit(failed)
