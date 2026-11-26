#!/usr/bin/env python3
"""Run the production v3 codec under ASan/UBSan and emit C-generated JSON.

Only temporary SDK shims are used; the production state layout is required.
"""
import json
import os
from pathlib import Path
import shlex
import struct
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
fixtures = root / "tests/fixtures/batch_v3.json"
headers = {
    "esp_err.h": "#pragma once\ntypedef int esp_err_t;\n",
    "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "esp_system.h": "#pragma once\n#include <stdint.h>\nuint32_t esp_get_free_heap_size(void);\n",
}
with tempfile.TemporaryDirectory(prefix="glove-batch-packets-") as temp:
    directory = Path(temp)
    for name, contents in headers.items():
        (directory / name).write_text(contents)
    for name, channels, raw, dummy in [("c3_raw", 3, True, False),
                                       ("s3_dmp", 8, False, False),
                                       ("c3_raw_dummy", 1, False, True),
                                       ("c3_disabled_aux", 0, True, False)]:
        config = {
            "CONFIG_TARGET_BOARD_XIAO_ESP32_C3": int(name.startswith("c3")),
            "CONFIG_AUX_ADC_ENABLED": int(channels > 0),
            "CONFIG_AUX_ADC_CHANNEL_COUNT": channels,
            "CONFIG_IMU_RAW_ENABLED": int(raw),
            "CONFIG_IMU_RAW_DUMMY": int(dummy),
        }
        (directory / "sdkconfig.h").write_text("#pragma once\n" + "".join(
            f"#define {key} {value}\n" for key, value in config.items()))
        binary = directory / name
        subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-g", "-O1", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            "-fno-omit-frame-pointer", f"-I{directory}", f"-I{root / 'main'}",
            str(root / "main/ble/ble_batch_packets.c"),
            str(root / "tests/native/batch_packets_regression.c"), "-o", str(binary),
        ], check=True, timeout=60)
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":halt_on_error=1:abort_on_error=1"
        env["UBSAN_OPTIONS"] = env.get("UBSAN_OPTIONS", "") + ":halt_on_error=1:print_stacktrace=1"
        args = [str(binary)]
        if name == "c3_raw":
            fixtures.parent.mkdir(parents=True, exist_ok=True)
            args.append(str(fixtures))
        print(f"Batch codec: {name}", flush=True)
        subprocess.run(args, check=True, env=env, timeout=30)

# Check the handoff file's JSON syntax, frame/fragment linkage and size bounds.
data = json.loads(fixtures.read_text())
assert len(data["cases"]) == 4
for case in data["cases"]:
    frame = case["frame"]
    parts = case["fragmentsMtu23"]
    assert len(frame) <= 512 and all(8 < len(part) <= 20 for part in parts)
    assert frame == [byte for part in parts for byte in part[8:]]
    assert all(part[:2] == [243, case["kind"]] and part[4:6] == [i, len(parts)]
               for i, part in enumerate(parts))
print(f"C-generated Dart cross-validation fixtures: {fixtures}")

# Independent little-endian decoding checks each fixture against the C input
# expectations, rather than merely trusting two consumers of the same bytes.
for case in data["cases"]:
    frame = bytes(case["frame"])
    kind = case["kind"]
    assert frame[:3] == bytes([3, kind, 1])
    if kind < 4:
        slot_size = {1: 24, 2: 22, 3: 20}[kind]
        seq, timestamp, rate, fmt, reserved = struct.unpack_from("<IIHBB", frame, 4)
        assert reserved == 0 and frame[16:28] == bytes(12)
        assert len(frame) == 28 + frame[3] * slot_size * 2
        assert len(case["samples"]) == frame[3]
        for i, sample in enumerate(case["samples"]):
            pos = 28 + i * slot_size * 2
            ds, dt, flags = struct.unpack_from("<BHB", frame, pos)
            assert sample["gloveId"] == 0
            assert sample["sequence"] == (seq + ds) % (1 << 32)
            assert sample["timestampMs"] == (timestamp + dt) % (1 << 32)
            assert sample["flags"] == flags and sample["rateHz"] == rate
            assert frame[pos + slot_size:pos + 2 * slot_size] == bytes(slot_size)
            if kind == 1:
                assert list(struct.unpack_from("<10h", frame, pos + 4)) == sample["values"]
                assert fmt == sample["accelRange"] | (sample["gyroRange"] << 2)
            elif kind == 2:
                assert list(struct.unpack_from("<4i", frame, pos + 4)) == sample["q30"]
                assert struct.unpack_from("<H", frame, pos + 20)[0] == sample["accuracy"]
                assert fmt == 0
            else:
                assert list(struct.unpack_from("<8h", frame, pos + 4)) == sample["values"]
                assert fmt == data["active_aux_channels"]
    else:
        assert len(frame) == 112 and frame[3] == 1
        assert frame[56:108] == bytes(52) and frame[110:112] == bytes(2)
        fields = [("uptimeMs", "I", 0), ("requestedRateHz", "H", 4),
                  ("actualRateHz", "H", 6), ("notificationRateHz", "H", 8),
                  ("batteryMv", "H", 10), ("mtu", "H", 12), ("boardId", "B", 14),
                  ("imuState", "B", 15), ("batteryPercent", "B", 16),
                  ("flags", "B", 17), ("phy", "B", 21), ("featureFlags", "H", 22),
                  ("freeHeapBytes", "I", 24), ("droppedNotifications", "I", 28),
                  ("fifoOverflows", "I", 32), ("i2cErrors", "I", 36),
                  ("droppedSensorSamples", "I", 40), ("rawSampleCount", "I", 44),
                  ("lastError", "i", 48)]
        for name, fmt, offset in fields:
            assert struct.unpack_from("<" + fmt, frame, 4 + offset)[0] == case["expected"][name], name
        assert struct.unpack_from("<H", frame, 108)[0] == case["expected"]["adcNotificationRateHz"]
        assert list(frame[22:25]) == case["expected"]["firmware"]
print("PASS: independent fixture decoding matches every expected sensor/status field")
