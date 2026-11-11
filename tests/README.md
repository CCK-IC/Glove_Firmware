# Firmware regression checks

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

# Hardware configuration and protocol

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
- Select hardware raw/DMP modes independently from their dummy alternatives.
  Enabling DMP selects the component's required DMP support. The configured
  build remains in its existing dummy mode until explicitly changed.

Hardware validation is still required for channel wiring, ADC accuracy, sensor
orientation and scale, DMP FIFO behavior, and BLE delivery under load.
