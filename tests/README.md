# Firmware regression checks

Run `python3 tests/native/run.py` on macOS with a C compiler. This compiles the
actual packet encoder and state manager with minimal host SDK shims, using
AddressSanitizer and UndefinedBehaviorSanitizer. It checks AUX buffer guards,
MTU/capacity boundaries, all eight channel fields, signed quaternion encoding,
and two concurrent writers incrementing shared counters.

After configuring an ESP-IDF build, run `python3 tests/check_configs.py`.
It uses `build/compile_commands.json` and temporary sdkconfig headers to check
application source syntax for the current configuration, real raw IMU, DMP with
fallback, real ADC, and disabled sensors. It does not run Kconfig resolution,
link alternate configurations, or establish that the test GPIOs are valid wiring.
Run `cmake --build build` for the configured target's complete build.

# Hardware configuration and protocol

- AUX V1 is **22 bytes**: version u8, sequence low u8, timestamp_ms u32 LE,
  then eight signed 16-bit LE values. Notifications require ATT MTU >=25;
  smaller MTUs are rejected without writing outside the packet. The previous
  20-byte declaration lost the eighth channel and overran the buffer.
- For real AUX input, disable `ADC_DUMMY`, set `AUX_ADC_GPIO_0` through
  `AUX_ADC_GPIO_7`, and choose `ADC_AVERAGE_SAMPLES`. Values are raw ADC counts.
  Every pin must be ADC-capable, distinct, and separate from battery, I2C,
  LED, button, and power-hold pins. Unconfigured pins default to -1 and cause
  initialization to fail rather than returning fabricated measurements.
  The board must expose eight usable ADC inputs; an external ADC or multiplexer
  needs a separate driver and wiring definition. This repository supplied neither.
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
