# Firmware handoff

## Publication branches, 2026-11-20

- Firmware 1.4.1 remains on `codex/v1.4.1` (release commit `67b2436`).
  Companion app 1.1.0+2 uses `codex/stable-v3-fw-v1.4.1` for its Stable V3
  release (app commit `dfeddf6`), including v3 batch ingestion and independent
  ADC rate control.
- The subsequent [firmware refactor review](REFACTOR_REVIEW.md) records measured
  build comparisons and future recommendations. It does not apply those
  optimizations or establish new radio throughput results.

## I²C recovery and ADC rate — firmware 1.4.1, 2026-11-19

- The first IMU initialization uses the 1.4.0 sequence and configured clock.
  After a startup failure, the firmware starts BLE and ADC and retries IMU
  initialization up to three times at a clock capped at 100 kHz.
- Runtime I²C recovery arms after the first valid hardware sample. A timeout
  or five consecutive failures by default schedules a bus reset. The firmware
  verifies the IMU's identity and power state, then attempts full
  reinitialization if that check fails. BLE and ADC remain active.
- Protocol v3 control opcode `0x08` sets the ADC rate independently of the
  IMU. The device reports the measured ADC rate with AUX samples.
- On the XIAO ESP32-C3, a full power cycle restored IMU startup after repeated
  register failures. A later run observed failed register reads and a
  bank-select write, followed by resumed BLE motion samples on the same boot.
  The stream also survived a 45/90/45 Hz rate change and a reconnect. Measured
  IMU rates fell below the request at 90 and 100 Hz. Stuck-bus recovery still
  needs a controlled hardware test.

## Fixed-slot sensor batches — firmware 1.4.0, 2026-11-16

- The 1.4.0 validation used working changes based on `efa7c56` and companion
  app version 1.1.0+2.
- Raw IMU, Q30 quaternion, AUX and status now use protocol v3, with unchanged
  UUIDs and commands. Both glove slots are physically present; reserved slot1
  metadata/payload are zero with presence mask1. No interglove communication
  is implemented. [Wire specification](docs/BATCH_PROTOCOL_V3.md).
- Preferred MTU 247: four raw records in 220 bytes. Batches retain exact u32
  timestamps/sequences and compact deltas; queues capture every accepted sample
  instead of periodically reading only latest caches. All accepted DMP FIFO
  quaternions are published after the device lock is released. Existing
  magnetometer initialization/recovery remains intact.
- Queues hold 64 records per raw/DMP/AUX stream. Flush at 80 ms or capacity;
  overflow drops oldest unread samples with a counter. NimBLE-host events
  serialize enqueue against GAP/CCC changes, retain transient failures for up
  to 500 ms, and rotate five streams with a six-attempt work budget. Epoch and
  connection generation fences prevent stale replay. MTU 23 uses ordered
  fragments; payload sizes and reassembly bounds are documented.
- Independent review added deterministic sender race tests. A live-test
  measured-rate metadata rejection was fixed by splitting raw batches before
  rate changes; regression coverage includes that boundary.
- Full C3 raw and managed DMP+raw builds and all ten feature syntax variants
  pass. The board retains the tested raw image. Codec, sample
  queue, sender, ADC/state and raw/DMP magnetic regressions pass ASan/UBSan.
  Cross-language C goldens validate all fields through Dart decoders and MTU 23
  reassembly. App: 110 VM tests and 45 selected Chrome tests pass, with clean
  analysis and a successful web release build.
- Final raw image flashed to `XIAO-IMU-RAW-75F2`; MTU 247 live test at requested
  45/90/200/45 Hz (15 sec each) received 3,632 raw samples/963 notifications and
  5,870 dummy ADC samples/1,174 notifications. No gaps, malformed frames,
  disconnects or new notification/sample/FIFO/I2C losses. All magnetic valid
  flags and absent-slot/inactive-channel zeros passed. Resubscribe and
  reconnect tests received only fresh records and no sequence gaps.
- [Exact hardware results](tests/hardware/batch_v3_1.4.0_2026-11-16.json) record
  binary SHA256. [Validation scope](docs/VALIDATION_V3.md) distinguishes physical
  raw IMU from dummy AUX. Physical DMP, electrical ADC, MTU 23 hardware and
  Windows and other OS BLE runs remain unverified.

## Magnetometer startup recovery — firmware 1.3.1, 2026-11-16

- Hardware diagnosis confirmed SLV4 remained armed (`CTRL=0x80`) with
  `MST_STATUS=0x00` when `I2C_MST_CYCLE` was cleared. Longer timeouts, auxiliary
  resets and a gyro-filter change did not restore completion. Selecting
  `SAMPLE_MODE_CYCLED` for the auxiliary master restored identity `0x4809` on
  the first attempt, with the original gyro settings retained.
- The AK09916 still runs in continuous 100 Hz measurement mode (`CNTL2=0x08`)
  for raw sampling. Cycled master scheduling controls the ICM's auxiliary
  engine, separately from the magnetometer's measurement mode. DMP setup
  retains ownership of its SLV0/SLV1 layout.
- Startup allows 50 ms per SLV4 byte, separately from the streaming 5 ms byte /
  20 ms poll budgets. Failed identity probes retry at most three times, with
  a checked auxiliary reset, re-enable and 10 ms settling interval. Identity
  and mode checks remain strict. Error logs include the failing call/register,
  phase, MST status and elapsed time. Temporary register dumps were removed.
- The bounded reset/retry follows the
  [upstream startup sequence](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/src/ICM_20948.cpp).
  Cycled scheduling is backed by the attached board's comparison test, rather
  than an assumption that a longer timeout resolves this failure.
- Verification on the USB-connected XIAO ESP32-C3 (`XIAO-IMU-RAW-75F2`):
  the final 1.3.1 image passed three consecutive boots, each finding identity
  `0x4809` on attempt 1, accepting raw mode `0x08` and starting the app.
  Ten-second diagnostic windows reported no raw drops or I2C/FIFO errors.
- A 30-second macOS BLE test negotiated MTU 128 and received 1,616 full raw
  V1 packets. All 1,616 had MAG_VALID set (`flags=0x1f`), none had overflow,
  and none contained an all-zero vector. There were 631 distinct magnetic
  vectors; X/Y/Z count ranges were 233..266, 30..69 and -271..-240. The
  connection stayed active; dropped-notification, FIFO and I2C counters stayed
  at zero. This validates delivery/validity, rather than magnetic calibration.
- Both complete ESP32-C3 raw and managed-DMP+raw builds passed, alongside
  the ten configuration checks and raw/DMP ASan/UBSan suites. The regression
  bus model reproduces the continuous-master stall; restoring that mode as a
  mutation makes the test fail. DMP on physical hardware remains untested.
- Validation details are stored in
  [tests/hardware/magnetometer_1.3.1_2026-11-16.json](tests/hardware/magnetometer_1.3.1_2026-11-16.json).


## Magnetometer validity — firmware 1.3.0, 2026-11-11

- Branch `codex/v1.3.0` starts from `codex/v1.2.0` (`e67e90b`). The BLE
  status version and ESP-IDF image metadata are 1.3.0.
- Raw mode disables the autonomous SLV0/SLV1 reads/writes and uses checked
  SLV4 transactions to read AK09916 ST1, then HXL through ST2. This prevents
  background reads from clearing DRDY before the host observes it. Identity
  and measurement mode are verified during initialization.
- A complete, non-overflow vector stays valid for less than 100 ms. No-data
  polls retain that vector without extending its timestamp; overflow, I2C
  failure, reset and sleep invalidate it. Partial reads release ST2 before
  retrying. A failed transaction must confirm SLV4 is disabled before reuse,
  and failed bank selections invalidate the driver's bank cache. Completion
  polling is limited to 5 ms per byte and 20 ms per poll;
  a blocking host I2C transfer is additionally subject to the driver timeout.
- DMP keeps its own auxiliary-slave layout and single-shot mode. Raw packets
  use uncalibrated Compass FIFO counts instead of decoding that incompatible
  layout as normal AGMT data. FIFO draining is bounded; incomplete frames and
  undrained backlogs reset the FIFO and invalidate magnetic data. Transport
  errors also reset it; a failed reset blocks further frame reads until recovery.
  `IMU_ERROR_REINIT_COUNT` now controls the degraded-status threshold; frame
  resynchronization is immediate after any destructive read error.
- BLE packet formats are unchanged. Magnetic data needs full raw V1 (36-byte
  payload, ATT MTU at least 39); compact raw V2 carries no magnetic values.
- Verification: complete ESP32-C3 raw and managed-DMP+raw builds passed,
  with version 1.3.0 embedded. Existing packet/state/ADC regressions, ten SDK
  syntax variants, and new raw/DMP magnetometer sanitizer tests passed.
- Hardware verification remains required: flash 1.3.0, confirm the status
  version, use real raw mode with full V1 packets, rotate the board and confirm
  magnetic validity and changing signed values. Check DMP+raw separately,
  including count scale/axis signs and FIFO recovery. The build alone does not
  establish that the physical sensor is functioning.


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
