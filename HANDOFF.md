# Firmware handoff

## C3 AUX channel limit, 2026-11-11

- Added `AUX_ADC_CHANNEL_COUNT`: 1–3 inputs on C3 (default 3), 1–8 on S3
  (default 8). Menuconfig exposes only the configured GPIO assignments. C3
  real-input GPIOs default to -1 and require explicit wiring configuration.
- Acquisition samples only those inputs. Real and dummy samples zero unused
  fields, and the AUX encoder also masks unused slots defensively. AUX V1
  remains 22 bytes with eight signed 16-bit values, so existing clients can
  display the unused channels as zero.
- The default C3 battery pin is GPIO2 and power-hold pin is GPIO5. GPIO5 also
  uses ADC2, which ESP-IDF restricts on C3. The maximum count does not guarantee
  three usable real inputs under a particular wiring configuration.
- The configured ESP32-C3 firmware build passed using CMake. Sanitizer-backed
  packet/state/ADC tests passed for C3 counts 1 and 3, S3 count 8, and disabled
  AUX. Ten SDK syntax variants and focused C3/S3 Kconfig resolution checks
  passed. GPIO wiring and BLE delivery have not been tested on hardware.


For the subsequent refactor and current review findings, see
[REFACTOR_REVIEW.md](REFACTOR_REVIEW.md). The snapshot below records the
pre-refactor history and validation results.

Snapshot prepared 2026-11-07. This document describes the history through
`4a2dcc6` and the changes committed alongside this handoff. Git history remains
the authoritative record of exact changes and merge ancestry.

## Project overview

ESP-IDF firmware for a BLE sensor glove. `main/app_main.c` initializes shared
state, board GPIOs, power, LEDs, battery, IMU, AUX ADC, BLE, buttons and
diagnostics. The BLE peripheral streams sensor data and accepts controls; see
`SerialMSG.md` for example serial logs and `tests/README.md` for protocol,
configuration and regression-check details. Board and feature settings live in
`main/Kconfig.projbuild` and defaults in `sdkconfig.defaults`. The generated
`sdkconfig` and build outputs are local, ignored files.

BLE names use `XIAO-IMU-RAW-` plus the last four hexadecimal digits of the
Bluetooth MAC. This makes nearby devices easier to distinguish, but two devices
can still share the same four-digit suffix. A single BLE connection is configured
by default.

## Commit history

Listed in Git's reverse chronological traversal order (oldest first); merge
commits preserve parallel development and are not additional feature releases.

| Commit | Summary |
| --- | --- |
| `28ad9ba` | Modified initial example. |
| `5bde73f` | Merged BLE firmware work, retaining raw IMU data. |
| `837b312` | Adjusted the IMU polling limit. |
| `a370050` | Added the ADC manager and dummy ADC streaming; real polling was work in progress. |
| `1a1356d` | Began dummy IMU support. |
| `f716687` | Completed a dummy IMU path and refactored BLE subscription handling and diagnostics. |
| `848bcc7` | Added serial message documentation. |
| `f97e7c8` | Refactored sensor firmware, BLE streaming and diagnostics; added regression checks. |
| `38ba9c2` | Appended a MAC-derived suffix to the advertised device name. |
| `0e265ed` | Updated board and IMU Kconfig options and fixed an IMU compile issue. |
| `2900a07` | Merged parallel `main` history. |
| `2c8872a` | Ignored and stopped tracking local VS Code configuration. |
| `b6fe1e4` | Merged parallel `main` history. |
| `6e3ce08` | Merged remote-tracking `origin/main`. |
| `4a2dcc6` | Updated power-off timing/logging and Kconfig settings. |

## Changes included with this handoff

- The power manager shuts down after a continuous disconnected interval,
  including the initial period after boot. `CONFIG_POWER_DISCONNECTED_TIMEOUT_SEC`
  defaults to 30 seconds; `0` disables it. A connection cancels the countdown,
  and a later disconnect starts a new one. Connection transitions and the
  shutdown claim are synchronized with shared state.
- The monitor starts after application initialization. If initialization exits
  early, it will not run. The existing shutdown sequence displays the configured
  LED indication, stops streaming and the IMU, then releases the power latch.
  `CONFIG_POWER_LATCH_TEST_MODE` retains the latch and uses deep sleep instead;
  the project defaults disable test mode.
- Existing local AUX ADC work creates per-channel calibration handles and logs
  calibration failures. The sample path still reports raw averaged ADC counts;
  it does not apply those handles to convert values to voltage. Battery code
  also comments out an unused calibration variable.

## Verification and open items

- `cmake --build build -j 4` passed for the locally configured ESP32-C3 build.
  The timeout change was also built in an ESP32-S3 configuration. Build results
  do not establish correct wiring or physical latch behavior.
- `python3 tests/native/run.py` passed its packet bounds and concurrent state
  checks. It does not exercise the new timeout on hardware.
- `python3 tests/check_configs.py` passed current, raw, ADC and disabled syntax
  variants. Its DMP variant failed because `imu_manager.c` references
  `CONFIG_IMU_DMP_OVERIDE_RATE_HZ`, which is absent from the generated test
  configuration. Check the intended Kconfig option and DMP profile before
  enabling that path; the matrix is a syntax check, not a full alternate build.
- Test boot without a connection, reconnect before the deadline, disconnect
  again, and verify physical power removal on the target board. The timeout is
  checked at 250 ms intervals and the power-off LED delay follows it.
- Validate real ADC pin mapping and calibration, battery readings, IMU/DMP
  operation, and BLE delivery under load on the intended hardware. AUX V1 is
  22 bytes and requires ATT MTU at least 25.
- Prior Windows BLE notification timeouts have not been demonstrated resolved
  on hardware by this handoff. Recheck connection and sustained streaming with
  the intended Windows clients and adapters.

For a fresh setup, install a compatible ESP-IDF, select the intended target and
board, configure the project using `idf.py menuconfig`, then run `idf.py build`.
Avoid reusing an existing `sdkconfig` or build directory across targets without
reconfiguring them. See `tests/README.md` for the full regression commands.
