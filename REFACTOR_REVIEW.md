# Firmware refactor and review — 2026-11-07

Working changes based on `1f158dd` (`codex/v1.1.0`) report firmware version
`1.2.0`. BLE UUIDs, command values, packet layouts and packet versions are
preserved. The version branch remains the pre-refactor rollback point. No
hardware performance claim is made from the build or syntax checks.

## Implemented

- Clarified shared-state names: `requested_imu_rate_hz`, `dmp_sample_count`,
  `dropped_dmp_samples` and `actual_imu_notify_rate_hz`. The latter counts
  successful DMP and raw IMU notification enqueues, not AUX/status packets or
  acknowledgements from the client.
- Clarified APIs such as `imu_manager_latest_dmp`, `imu_manager_reset_fifo`,
  `gatt_dmp_subscribed` and `power_manager_request_shutdown`. Named control
  commands and shutdown reasons replace magic numbers; redundant power APIs,
  obsolete declarations and commented-out implementations are removed.
- Renamed `IMU_DMP_OVERIDE_RATE_HZ` to `IMU_DMP_OVERRIDE_RATE_HZ` with
  `main/sdkconfig.rename` migration for existing configurations. DMP syntax
  checks no longer require a rate option from an inactive profile.
- Serialized IMU driver access; shutdown waits for sampling to finish before
  sleeping the device. Initialization failures detach the I2C device. Sample
  rates use elapsed time, deadlines retain phase, and raw acquisition respects
  its requested rate independently of DMP FIFO polling. DMP dummy generation
  and its GATT characteristic now work together.
- Real raw magnetometer flags check data-ready and overflow bits. During DMP
  operation, raw magnetic values are not marked valid because the managed
  driver's normal AGMT decoder expects a different external-sensor layout.
- BLE subscription getters no longer write shared state. Atomic subscription
  flags and a connection mutex synchronize host and notification tasks.
  Pairing deadlines expire, reconnect/reset clears stale connection data, AUX
  delivery is no longer throttled by a slower IMU stream, idle polling is
  reduced, and repeated notification errors are rate-limited. GATT commands
  report event-queue failures instead of silently claiming success.
- Battery ADC/conversion failures retain the last good measurement and expose
  the error. Successful voltage/percentage/low/error fields publish together.
  Removed unused AUX calibration objects: AUX continues to send averaged raw
  counts. Board validation also rejects input-only GPIOs assigned to outputs.
- Shutdown claims are atomic, with separate manual/disconnected entry points.
  The disconnected monitor also starts if IMU initialization returns early.
  The 30-second default and zero-to-disable behavior are unchanged.
- LED pattern changes restart their phase immediately; unchanged colors avoid
  redundant hardware transfers. Expiration uses 64-bit time, debug/test
  patterns can override normal status, and LED initialization failures clean
  up allocated resources. Pairing indication has a single status-task owner.
- Disabled periodic diagnostics creates no task. Button events report queue
  failures, and a button already held at task startup gets a valid hold epoch.

## Verification

- `cmake --build build -j 4`: ESP32-C3 firmware builds successfully.
- `python3 tests/native/run.py`: sanitizer-backed packet bounds, concurrent
  shared-state updates and shutdown timing/claim tests pass.
- `python3 tests/check_configs.py`: current, raw, DMP, ADC, dummy,
  power-disabled and sensor-disabled syntax variants pass. These are syntax
  variants, not linked alternate firmware images or hardware validation.
- `git diff --check`: clean.

## Remaining improvements

| Priority | Evidence and consequence | Suggested next work |
| --- | --- | --- |
| High for deployed devices | The control characteristic in `main/ble/gatt_server.c` permits writes without encryption/authentication flags or an application authorization check. A connected untrusted peer can request shutdown, change rates or start pairing; the shutdown magic bytes are not authentication. | Define the production pairing/ownership policy, then enforce it on control writes and test Windows/macOS interoperability. |
| Medium | `main/battery/battery_manager.c` falls back to `raw * 3300 / 4095` when calibration is unavailable, regardless of target and attenuation. Battery percentage and low-battery indication can be inaccurate. | Require valid calibration for production measurements, or add a measured board/attenuation-specific fallback and expose measurement quality. |
| Medium | Sampling and notification tasks use whole RTOS ticks. The local `CONFIG_FREERTOS_HZ=100` permits at most one poll per 10 ms, while project options accept up to 200 Hz. Phase preservation avoids some rounding losses but cannot exceed that ceiling. | Measure the required rates and CPU load, then choose a higher tick rate or timer-driven task notifications; drain DMP FIFO appropriately. |
| Medium | BLE operations use a connection-handle snapshot before entering NimBLE. Disconnect/reconnect can occur before the operation, and handles may be reused. | Dispatch operations on the host task with connection-generation validation and exercise rapid reconnects. |
| Medium | Several managers still assume a single init/start lifecycle; partial BLE startup and repeated sensor initialization do not have complete rollback/cleanup. | Add explicit lifecycle ownership and failure cleanup before supporting runtime reinitialization. |
| Medium | Most application startup failures still use `ESP_ERROR_CHECK`, which may reboot repeatedly. The nonfatal IMU failure path now keeps the timeout active, but this is not a complete startup recovery policy. | Decide which failures warrant retry, degraded service or power-off, and implement one consistent recovery path. |
| Medium | There are no hardware regression results for the refactor. | Verify disconnected latch release, sustained Windows/macOS streams, battery accuracy, sensor scales and DMP FIFO behavior on each supported board. |

Further performance work should follow measurements of I2C utilization, task
stack high-water marks, heap use and actual BLE packet rates. Splitting the
large GATT command/attribute callback into handlers is a useful next readability
step, but should preserve its ATT errors and packet contracts.
