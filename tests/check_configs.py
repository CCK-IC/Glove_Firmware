#!/usr/bin/env python3
"""Check feature branches without changing the configured build or sdkconfig."""
import argparse
import json
import re
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--build-dir", type=Path, default=root / "build",
                    help="Configured ESP-IDF build; external builds reuse SDK headers/toolchain")
parser.add_argument("variants", nargs="*", help="Optional variant names (default: all)")
args = parser.parse_args()
build = args.build_dir.resolve()
entries = json.loads((build / "compile_commands.json").read_text())
base = (build / "config/sdkconfig.h").read_text()
# A clean archive can check its own sources against an existing configured SDK.
# Keep SDK/generated includes and compiler response files in that build intact.
description = build / "project_description.json"
source_root = Path(json.loads(description.read_text())["project_path"]) if description.exists() else root
is_c3 = bool(re.search(r"^#define CONFIG_IDF_TARGET_ESP32C3 1$", base, re.M))
channel_limit = 3 if is_c3 else 8
pins = [0, 1, 3] if is_c3 else [2, 3, 7, 8, 9, 10, 11, 12]


def adc_config(count, dummy=False):
    changes = {"CONFIG_AUX_ADC_ENABLED": 1,
               "CONFIG_AUX_ADC_CHANNEL_COUNT": count,
               "CONFIG_ADC_DUMMY": 1 if dummy else None,
               "CONFIG_ADC_WIDTH_BITS": 12,
               "CONFIG_ADC_RATE_HZ": 100,
               "CONFIG_ADC_AVERAGE_SAMPLES": None if dummy else 2}
    for attenuation in ("0DB", "2_5DB", "6DB", "12DB"):
        changes["CONFIG_ADC_ATTEN_" + attenuation] = 1 if attenuation == "12DB" and not dummy else None
    # Absence matters: do not let stale build-header GPIO definitions hide
    # production references to inactive slots or dummy-only hidden options.
    for index in range(8):
        changes[f"CONFIG_AUX_ADC_GPIO_{index}"] = pins[index] if not dummy and index < count else None
    return changes

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
 'adc': adc_config(channel_limit),
 'adc_one': adc_config(1),
 'adc_dummy': adc_config(channel_limit, dummy=True),
 'adc_dummy_one': adc_config(1, dummy=True),
 'dummy': {**adc_config(channel_limit, dummy=True),
           'CONFIG_IMU_ENABLED': None,
           'CONFIG_IMU_DMP_ENABLED': None,
           'CONFIG_IMU_RAW_ENABLED': None,
           'CONFIG_IMU_DMP_DUMMY': 1,
           'CONFIG_IMU_RAW_DUMMY': 1,
           'CONFIG_IMU_SYNTHETIC_FALLBACK': None},
 'power_disabled': {'CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC': 0,
                    'CONFIG_POWER_LATCH_TEST_MODE': 1,
                    'CONFIG_DIAGNOSTICS_LOG_PERIOD_MS': 0},
 'disabled': {**{f'CONFIG_AUX_ADC_GPIO_{i}': None for i in range(8)},
              **{f'CONFIG_ADC_ATTEN_{a}': None for a in ('0DB', '2_5DB', '6DB', '12DB')},
              'CONFIG_AUX_ADC_CHANNEL_COUNT': None,
              'CONFIG_ADC_WIDTH_BITS': None,
              'CONFIG_ADC_AVERAGE_SAMPLES': None,
              'CONFIG_AUX_ADC_ENABLED': None,
              'CONFIG_ADC_DUMMY': None,
              'CONFIG_IMU_RAW_DUMMY': None,
              'CONFIG_IMU_DMP_DUMMY': None,
              'CONFIG_IMU_RAW_RATE_HZ': None,
              'CONFIG_ADC_RATE_HZ': None,
              'CONFIG_IMU_SLOW_RATE_HZ': None}}

unknown = set(args.variants) - variants.keys()
if unknown:
    parser.error("Unknown variants: " + ", ".join(sorted(unknown)))

failed = False
for name, changes in variants.items():
    if args.variants and name not in args.variants:
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
            if not Path(entry["file"]).is_relative_to(source_root / "main"):
                continue
            compiler_args = [arg.replace(str(source_root / "main"), str(root / "main"))
                             for arg in shlex.split(entry["command"])]
            command = [compiler_args[0], "-I" + directory]
            index = 1
            while index < len(compiler_args):
                if compiler_args[index] in ("-o", "-MF", "-MT", "-MQ"):
                    index += 2
                    continue
                if compiler_args[index] in ("-c", "-MD", "-MMD", "-MP"):
                    index += 1
                    continue
                command.append(compiler_args[index])
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
