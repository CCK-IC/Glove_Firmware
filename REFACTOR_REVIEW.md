# Firmware refactor review

## Implemented refactor, 2026-11-24

The configured raw IMU and one-channel AUX ESP32-C3 build uses 102,439 bytes of statically linked internal RAM, down 5,368 bytes from 107,807. The application binary fell from 584,208 to 583,584 bytes with the same build configuration. Application C sources and headers went from 5,470 to 5,010 physical lines. The historical backup and Make component file accounted for another 278 lines outside that count.

- BLE notifications skip the full system-state snapshot in the GAP callback. Disconnect and host reset share link cleanup. GATT uses a common stream-to-handle mapping and keeps a separate snapshot per readable characteristic. [GAP and sender](main/ble/ble_manager.c) · [GATT server](main/ble/gatt_server.c)
- The sender and GATT buffers use sizes derived from the protocol. The ring caches sequence and time deltas and only allocates DMP storage and its lock when the configured profile exposes DMP. The common sensor-serialization and fragmentation paths use 512 fewer stack bytes each in the C3 build. The v3 fixture is byte-for-byte identical. [Sample rings](main/ble/ble_sample_buffer.c) · [Codec](main/ble/ble_batch_packets.c)
- The compatibility tests retain the three exercised legacy encoders. The shared sample types and AUX channel assertions remain in the firmware. Both DMP profiles and the reserved v3 peer slot remain available. The IMU retry paths share bus reset and detach code while retaining their distinct startup and runtime reset policies. [Legacy tests](tests/native/legacy_ble_packets.c) · [IMU retry](main/imu/imu_manager.c)

The ESP32-C3 release, managed DMP, application DMP override and sensor-disabled builds linked, as did the ESP32-S3 raw build. All seven native regression suites, ten syntax variants, and C3/S3 AUX Kconfig checks passed. The Flutter decoder passed all four cases using the regenerated firmware fixture, including MTU 23 reassembly. Delivered rate, latency, and IMU boot and recovery on the device still need hardware verification.

## Initial source-length audit, 2026-11-24

At `75424b0`, the 39 tracked C sources and headers under `main/` total 5,470 physical lines, including comments and blank lines. The BLE manager (867) and IMU manager (825) account for 31%. Tests and their Python runners account for another 3,975 lines outside that count. Two small, independent changes are good starting points.

1. **Trim the active BLE callbacks.** The GAP handler snapshots the entire system state before its switch. Even a successful notification TX event takes the mutex and copies the state, then discards it. Use the event fields and local MTU directly. Apply the same change to GATT control writes, which already receive the requested values. Share the locked teardown operations for disconnect and host reset. Keep the disconnect-specific rate defaults, reset-specific pairing cleanup, subscription invalidation and disconnected timer semantics explicit. Compare delivered samples/s and notification drops at the same MTU and load. [GAP and sender](main/ble/ble_manager.c) · [GATT controls](main/ble/gatt_server.c) · [State lock](main/system_state.c)
2. **Retire historical code from the application tree.** The 273-line backup `main/main.c.bk` and 425-line v1/v2 encoder are recoverable from commit `75424b0`. Of eleven public legacy encoder functions, native regressions called three. The release ELF contained four v3 encoder symbols and zero legacy encoder symbols. Move the exercised encoders and their declarations beside the compatibility tests, keep the shared sample types, flags and AUX channel-count assertions in production, and label the compact-packet Kconfig setting as legacy. [Shared header](main/ble/ble_packets.h) · [Native callers](tests/native/regression.c) · [Build list](main/CMakeLists.txt)

For the next pass, use a shared stream identifier and a single mapping between characteristic handles and stream IDs for GATT reads, subscription updates and sender dispatch. Preserve the per-stream subscription generation that rejects stale queued samples after reconnect, along with stable read snapshots for long ATT reads. Cache each sample's sequence and timestamp deltas once when taking a batch from the ring. The current loop repeats the same reads and subtraction under the ring mutex. If the sender becomes its own source file, compile that file in the native harness, which currently extracts three text ranges from the BLE manager by function name. [GATT subscriptions](main/ble/gatt_server.c) · [Batch queue](main/ble/ble_sample_buffer.c) · [Sender harness](tests/native/run_sender.py)

The largest optional-code duplication is the application DMP initializer. Of its 302 lines, 292 match the locally installed driver in order after trimming whitespace. The differences include I²C-master enablement, sample dividers, gyro scale and three tuning coefficients. Keep both selectable profiles. Consolidation is worthwhile when a pinned driver exposes those differences as parameters at the correct points in its initialization sequence. Verify cold boot, FIFO output rate and compass samples for both profiles on hardware before replacing the override. The raw IMU boot and runtime I²C recovery sequences have different reset requirements and should retain separate entry conditions. [DMP override](main/imu/imu_dmp_application_override.c) · [IMU initialization and recovery](main/imu/imu_manager.c)

## Firmware 1.4.1 review, 2026-11-20

Reviewed release commit `67b2436`. Start with a reproducible board profile and right-sized BLE buffers. Then address notification scheduling and the time spent acquiring each physical sample. Each change can be tested independently against the same wire protocol.

### Measured build footprint

The working ESP32-C3 profile uses ESP-IDF 6.1, real raw IMU, one real AUX input on GPIO3, DMP disabled, a 100 Hz RTOS tick, and a preferred ATT MTU of 247. Isolated builds used the same application sources, toolchain and installed component versions. The reproduced baseline matched the release build's image size and static RAM totals.

| C3 build | Application binary | Linked internal RAM, including resident code | Image saving vs. baseline |
| --- | ---: | ---: | ---: |
| Current profile, `-Og` | 584,208 B | 107,807 B | — |
| Central, observer and GATT client disabled, `-Og` | 568,880 B | 107,799 B | 15,328 B |
| Same roles, optimized for size with `-Os` | 512,960 B | 99,881 B | **71,248 B (12.2%)** |

All three builds include peripheral advertising, GATT server, bonding and the existing packet pools. Of the 7,926 B reduction in linked internal RAM in the last build, 7,686 B comes from resident code. Dynamic BLE pools and FreeRTOS task stacks are additional allocations. The comparison used the same local component sources and confirmed the resolved role and optimization settings for each build. [Espressif binary size guide](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32c3/api-guides/performance/size.html)

### Recommended changes in implementation order

**1. Make the tested radio and board configurations reproducible.** The checked-in defaults enable real AUX. A fresh C3 board configuration requests three unmapped inputs. On S3 the default eight assignments include GPIO4, 5 and 6, which the same profile assigns to power hold and I²C. The ADC manager correctly rejects them and `app_main` checks that result before BLE starts. Supply explicit board-specific defaults, retain runtime pin validation, and use verified wiring or a deliberate AUX-disabled profile. Record the deployed timeout separately from the bench configuration, which disables auto-off. Pin or lock the tested ICM20948 0.3.0 and LED strip 3.1.1 dependencies. Put the measured BLE role and compiler settings in a release profile, with central/client roles available to a future inter-glove profile. [Board and AUX options](main/Kconfig.projbuild) · [ADC validation](main/adc/adc_manager.c) · [Startup](main/app_main.c) · [Component manifest](main/idf_component.yml)

**2. Allocate BLE storage by characteristic and enabled producer.** The current ELF contains the following independently identifiable buffers. Define capacities from the protocol constants and check them at compile time.

| Current C3 allocation | Replacement | Estimated payload/storage saving |
| --- | --- | ---: |
| Four 512 B GATT read snapshots | Single-record raw 76 B, AUX 68 B and status 108 B. Add 72 B when DMP is enabled. | 1,796 B |
| Five 512 B pending payloads | Raw 412 B, AUX 348 B, status 108 B and battery 1 B. Add 380 B for DMP. | 1,691 B |
| 512 B transmit payload | Largest currently encodable frame is 412 B | 100 B |
| Disabled DMP ring, 64 × 28 B | Instantiate when real or dummy DMP produces samples | 1,792 B |

The combined estimate is **5,379 B before alignment and descriptor changes**. Keep each readable characteristic's snapshot stable across ATT Read Blob offsets, each stream's pending frame independent, and the posted host job immutable until completion. Preserve the existing characteristic discovery and read behavior for each profile, all 64 queue entries per active stream, and the zero-filled second glove slot. Keep the protocol's 512 B ceiling independent of today's implementation sizes. As a second pass, a one-channel internal AUX record can retain its timestamp, sequence and active value in approximately 12 B instead of 24 B. The encoder still emits the eight-channel wire layout. [GATT read storage](main/ble/gatt_server.c) · [Pending and transmit storage](main/ble/ble_manager.c) · [Sample rings](main/ble/ble_sample_buffer.c) · [Wire layouts](docs/BATCH_PROTOCOL_V3.md)

**3. Wake the notification sender when it has work.** At the current 100 Hz tick, the task sleeps at least one tick after at most six submissions. Its theoretical ceiling is about 600 submissions/s. At MTU 23, each fragment carries 12 B of frame data. Full eight-row raw and AUX batches need 35 and 29 notifications respectively. At 100 *acquired* samples/s per stream that is **800 notifications/s**, before status, retries or radio limits. At MTU 247 the same streams can use four raw or five AUX records per notification, about **45 notifications/s combined** with full batches. The small-MTU case needs explicit load testing. [Sender loop](main/ble/ble_manager.c) · [Fragment format and batch limits](main/ble/ble_batch_packets.c)

Wake on a full batch, oldest-sample flush deadline, subscription or MTU change, or host completion. Keep a bounded fair drain across streams. After exhausting a budget with more work ready, schedule another drain while yielding to the host. On packet pool exhaustion, back off or wait for capacity. The existing path can allocate and free six mbufs in one pass trying to preserve its four-block ATT reserve. Retain that reserve, the 80 ms partial-batch flush, 500 ms retry expiry, exact loss accounting and the generation/epoch checks that prevent replay after reconnects. Prefer coalesced readiness signals to one cross-task wake per sample. [Sender regression tests](tests/native/sender_regression.c)

**4. Remove repeated work in serialization and synchronization.** Raw, DMP and AUX encoders create zeroed 512 B stack arrays and copy the result to the destination. Validate all inputs first, zero the actual frame length and encode into the destination. Use an overlap-safe move for fragment payloads before writing the header. Preserve unchanged output on rejection and the existing in-place fragmentation test. The compiled raw encoder uses a 576 B stack frame. Measure stack high-water marks under pairing, recovery and MTU 23 before reducing a task's reservation. On the hot path, move full 136 B system-state snapshots into the GAP cases that need them, use narrow synchronized reads for acquisition, and fetch only the epoch when the caller needs an epoch. Preserve coherent complete snapshots for status reads. [Codec](main/ble/ble_batch_packets.c) · [State](main/system_state.c) · [Fragment regression](tests/native/batch_packets_regression.c)

**5. Improve sample production before raising BLE rates.** Previous board measurements on 1.4.1 delivered about 44–45 Hz at a 45 Hz IMU request and 56–66 Hz in the observed 90/100 Hz runs. The real raw path calls the managed `get_agmt` routine on every sample. It reads 23 data bytes and three configuration registers. The application uses 14 of the data bytes for acceleration, gyro and temperature, and obtains magnetic data separately. Read a single 14 B burst and cache the verified scale settings after initialization, DMP configuration and recovery. At 100 successful samples/s this removes 900 unnecessary burst bytes and 300 configuration reads per second, plus associated bank switches. Preserve signed decoding, register error reporting and magnetic validity. [Raw acquisition](main/imu/imu_manager.c) · [Board observations](HANDOFF.md)

Instrument motion-burst duration, bank switches, magnetic poll duration and missed deadlines separately. The raw magnetic reader uses nine SLV4 byte transactions for a fresh vector. Each involves at least six host register operations, for at least 54 operations before extra polling and bank switches. Its aggregate deadline is 20 ms against a 10 ms period at 100 Hz. Consider a contiguous auxiliary read only after validating fresh-data detection, overflow, ST2 release after partial reads, and DMP ownership of the auxiliary slots. Keep the 1.4.0 initial startup sequence and 1.4.1 runtime I²C recovery through the acquisition changes. [Magnetometer reader](main/imu/magnetometer_reader.c) · [Recovery tests](tests/native/dmp_magnetometer_regression.c)

At a 100 Hz tick, the polling tasks cannot reliably schedule independent 5 ms hardware reads for a 200 Hz request. The ADC path also samples unconditionally at or above the tick rate. A late task can execute multiple reads back to back as `vTaskDelayUntil` catches up. Drive task notifications from real sample deadlines, perform I/O in the worker, and skip expired periods. Preserve independent ADC and IMU rate controls. Add a test that keeps the ADC task alive across an injected 100 ms stall and checks actual timestamps. [ADC scheduler](main/adc/adc_manager.c) · [Current ADC test](tests/native/adc_regression.c)

**6. Reduce background tasks and preserve optional behavior.** The current application requests 28,672 B across nine resident task stacks, plus 4,096 B for the NimBLE host. Disconnected auto-off has its own optional worker outside that total. The least invasive choices are to use the existing diagnostics-period setting in a lean profile (3,072 B stack) and feed LED state changes into the existing LED worker (2,048 B status polling stack). Retain a diagnostics-enabled profile for I²C investigations. Have the LED worker wait for a command or its next transition instead of waking at 50 Hz even for a steady color. Later, make GPIO edges wake the button worker and keep debounce, multi-click, boot-held and shutdown deadlines. Pairing, battery acquisition and power-off operations that can block belong in worker context. Measure minimum free heap and stack headroom before combining workers. [Status and LED tasks](main/app_main.c) · [LED patterns](main/led/led_manager.c) · [Buttons](main/button/button_manager.c) · [Power](main/power/power_manager.c) · [Diagnostics](main/diagnostics/diagnostics.c)

### Disabled features and source cleanup

- Keep the DMP managed and override profiles, dummy raw/DMP/AUX producers, synthetic fallback, board-specific pins, disconnected shutdown and protocol reservations. Use the same feature predicate for a producer and its backing storage. With all IMU sources off, skip the three IMU mutexes and 4,096 B acquisition stack, and omit I²C initialization when there is no hardware bus client. Preserve the existing public entry points, status semantics and GATT behavior. [IMU initialization](main/imu/imu_manager.c) · [Board initialization](main/board/board_config.c)
- Keep reserved event and Kconfig names such as single click and rate persistence. Document their implementation status. Audit `TEST_BLE`, `IMU_NOTIFY_DUPLICATE_SAMPLES` and `BLE_SECURE_CONNECTIONS` before reorganizing options. The last has no active application use, while `BLE_PREFER_SECURE_CONNECTIONS` configures the host. Make the distinction visible to users and keep existing bond/compatibility behavior. [Kconfig](main/Kconfig.projbuild) · [BLE initialization](main/ble/ble_manager.c)
- `main/main.c.bk` and the legacy `component.mk` sit outside the active CMake source list. Removing or archiving them is housekeeping with zero release-image saving. The ELF already excludes the legacy `ble_encode_*` functions. Keep the shared sample types and golden/compatibility tests if moving those encoders into a legacy module. Retain the DMP spelling migration in `sdkconfig.rename`. [Build sources](main/CMakeLists.txt) · [Shared types](main/ble/ble_packets.h) · [Legacy regressions](tests/native/regression.c)

### Verification and acceptance

- The release C3 build, seven native test runners, all ten syntax variants, and C3/S3 AUX Kconfig checks passed. The existing syntax test's `disabled` case inherits real IMU flags from the current build. It also compiles only files in the input compilation database. A raw build cannot cover the application DMP override source by header substitution alone. Exercise distinct resolved configurations and full links in CI. [Configuration checks](tests/check_configs.py) · [Test procedures](tests/README.md)
- Four additional configurations linked in temporary checkouts. The C3 with real and dummy IMU streams and AUX all disabled produced a 561,904 B image. The C3 raw plus managed DMP profile produced 611,344 B, and raw plus the application DMP override produced 611,360 B. The S3 raw configuration also linked. Its default AUX GPIO conflicts still require resolving before a board boot test. The regenerated protocol fixture matched the checked-in fixture byte for byte.
- Compare the release and candidate on the same board, radio client and acquisition configuration. First use dummy streams at fixed rates to measure transport capacity. Then use physical raw and DMP modes to measure acquisition and recovery. At MTUs 23 and 247, record requested, acquired and client-received samples/s, sequence gaps, actual notifications/s, oldest-sample delivery latency (median and p95), queue and notification drops, lowest free mbuf count, minimum free heap and task stack high-water marks.
- Include Windows native/Chrome subscription setup, macOS reconnect, GATT long reads, simultaneous raw/AUX/status, a midstream rate change, and transient I²C read/write errors. Check that real samples resume on the same boot and drop counters stop increasing after unsubscribe or disconnect. Use overload cases to verify fair control/status progress and accounted losses. Compare the six-job sender against the candidate at identical small-MTU load. Require repeatable improvement in delivered rate or latency and no regression in loss at loads the baseline handles. The reviewed size variants were bench builds. The glove was unavailable in this review's BLE scan, so radio throughput and latency gains remain to be measured.

## Historical review, 2026-11-07

Working changes based on `1f158dd` (`codex/v1.1.0`) report firmware version
`1.2.0`. BLE UUIDs, command values, packet layouts and packet versions are
preserved. The version branch remains the pre-refactor rollback point. No
hardware performance claim is made from the build or syntax checks.

### Implemented

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

### Verification

- `cmake --build build -j 4`: ESP32-C3 firmware builds successfully.
- `python3 tests/native/run.py`: sanitizer-backed packet bounds, concurrent
  shared-state updates and shutdown timing/claim tests pass.
- `python3 tests/check_configs.py`: current, raw, DMP, ADC, dummy,
  power-disabled and sensor-disabled syntax variants pass. These are syntax
  variants, not linked alternate firmware images or hardware validation.
- `git diff --check`: clean.

### Remaining improvements

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
