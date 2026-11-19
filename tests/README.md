# Firmware regression checks

Run `python3 tests/native/run_magnetometer.py` to test the production
magnetometer reader with the actual managed-component headers and a simulated
register bus. It requires the ICM20948 managed dependency to be installed.
The sanitizer-backed checks cover initialization, mode/identity checks,
DRDY/data/ST2 ordering, signed vectors, freshness, overflow, transaction errors,
timeouts, partial-read recovery and preservation of unrelated packet flags.
Startup checks additionally model delayed auxiliary completion, a master that
only recovers after reset, bounded persistent failures and strict identity/mode
rejection. Startup budgets are independent of streaming budgets. The bus model requires
cycled auxiliary scheduling: continuous master mode leaves SLV4 armed without
completion, matching the hardware regression.
Run `python3 tests/native/run_dmp_magnetometer.py` to check the actual IMU
manager's Compass-only and quaternion frames, latest-frame selection, raw DMP
publication, freshness, backlog/time budgets, transport errors, partial frames
and failed-reset gating under the same sanitizers.
These tests do not establish physical I2C timing or sensor operation.


Run `python3 tests/native/run.py` on macOS with a C compiler. This compiles the
actual packet encoder, state manager, and ADC manager with minimal host SDK
shims, using AddressSanitizer and UndefinedBehaviorSanitizer. It checks AUX
buffer guards, MTU/capacity boundaries, active and zero-filled channel fields,
real/dummy sampling, averaging, invalid pin mappings, failure recovery, disabled
ADC, signed quaternion encoding,
two concurrent writers incrementing shared counters, disconnected shutdown
deadlines, brief reconnections, and competing shutdown claims.

After configuring an ESP-IDF build, run `python3 tests/check_configs.py`.
It uses `build/compile_commands.json` and temporary sdkconfig headers to check
application source syntax for the current configuration, real raw IMU, DMP with
fallback, one/max-channel real and dummy ADC, dummy sensors, disabled
power/diagnostics, and disabled sensors. It does not run Kconfig resolution,
link alternate configurations, or establish that the test GPIOs are valid wiring.
Run `cmake --build build` for the configured target's complete build.
Run `python tests/check_aux_kconfig.py` with the ESP-IDF Python environment to
check C3/S3 channel limits and GPIO visibility, including old configurations,
dummy ADC, and disabled AUX.

## Protocol v3 and firmware 1.4.1 checks

Current sensor/status frames use fixed-slot batches; the layouts below labeled
V1/V2 document legacy formats only. See [v3 specification](../docs/BATCH_PROTOCOL_V3.md).
Run these additional suites with a C compiler supporting ASan/UBSan:

```sh
python3 tests/native/run_batch_packets.py
python3 tests/native/run_sample_buffer.py
python3 tests/native/run_sender.py
python3 tests/native/run_imu_i2c_transport.py
```

They exercise the production codec, bounded queues and sender functions with
host SDK/scheduler shims. `run_batch_packets.py` regenerates deterministic
`tests/fixtures/batch_v3.json`; copy it into the app's `test/fixtures/` and run
`flutter test --no-pub test/batch_cross_language_test.dart` there to verify C/Dart
wire compatibility. Native races model transient failures, connection/CCC
changes, MTU changes, deadline/completion interleavings and small-MTU fairness.
The I²C transport tests use the ESP-IDF managed ICM20948 dependency and inject
read timeouts and failed bank-select writes.

A manual live harness for the recorded 1.4.0 run requires Python with
`bleak` and platform Bluetooth permission. Close other BLE clients, use
firmware 1.4.0 **raw mode** with three AUX dummy channels at 100 Hz, and put
the board nearby. The harness subscribes to raw, AUX and status, requests
45/90/200/45 Hz with a 250 ms status period, and checks reserved zeros,
sequences, loss counters, subscription changes and reconnects. After the test
it requests 45 Hz with a 1,000 ms status period. Firmware restores its
configured rate after disconnect. Pass the exact advertising name.

```sh
python3 tests/hardware/check_batch_v3.py --name XIAO-IMU-RAW-75F2 --output batch_v3_live_result.json
```

This checks delivery and format, not calibrated sensor accuracy or all radio
conditions. It cannot substitute for Flutter UI and Windows/other platform
hardware tests. [Recorded release scope](../docs/VALIDATION_V3.md).

# Legacy hardware configuration and protocol

- AUX V1 is **22 bytes**: version u8, sequence low u8, timestamp_ms u32 LE,
  then eight signed 16-bit LE values. Notifications require ATT MTU >=25;
  smaller MTUs are rejected without writing outside the packet. The previous
  20-byte declaration lost the eighth channel and overran the buffer.
- Set `AUX_ADC_CHANNEL_COUNT` to the number of inputs to sample. ESP32-C3
  accepts 1–3 (default 3); ESP32-S3 accepts 1–8 (default 8). Only those GPIO
  assignments appear in menuconfig. In both real and dummy modes, remaining
  values in the eight-channel AUX packet are zero. Channel indices start at 0.
- For real AUX input, disable `ADC_DUMMY`, assign the visible `AUX_ADC_GPIO_*`
  settings, and choose `ADC_AVERAGE_SAMPLES`. Values are raw ADC counts.
  Pins must be ADC-capable, distinct, and separate from battery, I2C,
  LED, button, and power-hold pins. C3 mappings default to -1 so actual wiring
  must be configured. Unconfigured active pins fail initialization.
- The C3 defaults reserve GPIO2 for battery measurement and GPIO5 for
  power-hold. GPIO5 also uses ADC2, which ESP-IDF restricts on C3 due to
  hardware limitations. Select fewer AUX channels if three usable inputs are
  not available; do not reuse battery or power-hold pins. See the
  [ESP-IDF ADC oneshot limitations](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/adc_oneshot.html).
  The three-channel limit is a maximum, not a guarantee of available wiring.
- Battery and AUX acquisition share a single oneshot handle per ADC unit.
  Handle allocation occurs during serialized application initialization.
- Real IMU initialization retries actual identity reads, rejects an incorrect
  identity, and checks each setup operation. Raw packet scale codes come from
  the full-scale registers read by the IMU driver, including when DMP is active.
- Firmware 1.3.0 reads raw magnetometer registers through controlled SLV4
  transactions. DMP+raw uses uncalibrated FIFO Compass counts. Magnetic validity
  expires after 100 ms without a complete accepted vector, and resets/sleep or
  read failures invalidate it. Raw V1 is 36 bytes and requires ATT MTU >=39;
  compact V2 does not contain a magnetometer.
- Select hardware raw/DMP modes independently from their dummy alternatives.
  Enabling DMP selects the component's required DMP support. The configured
  build remains in its existing dummy mode until explicitly changed.

Hardware validation is still required for channel wiring, ADC accuracy, sensor
orientation and scale, DMP FIFO behavior, and BLE delivery under load.
