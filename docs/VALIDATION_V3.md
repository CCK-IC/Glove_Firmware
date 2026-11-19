# Batch protocol v3 validation

Release: app 1.1.0+2 / firmware 1.4.0. Branch in both repositories:
`codex/batched-sensor-packets-v1.4.0`. Base app commit `7f969dc`; base firmware
commit `efa7c56`. Results below apply to the working changes on 2026-11-16.

## Automated verification

- All 110 Flutter tests pass; `flutter analyze --no-pub` reports no issues.
- All 45 selected protocol/reassembly/timeline/ADC/diagnostics/chart tests also
  pass in Chrome, exercising JavaScript signed/unsigned arithmetic and u32 wrap.
- Browser release compilation passes. The macOS release build is blocked in
  Flutter's framework architecture verification: `lipo -info` reports both
  arm64 and x86_64, but `lipo -verify_arch` fails. Retrying with Xcode tools
  does not clear the build error. No native release-build success is claimed.
- Full ESP32-C3 raw and managed DMP-plus-raw firmware compile/link successfully
  with ESP-IDF v6.1 (raw 579,776 bytes; DMP 606,368 bytes). The raw image was
  installed for this validation. The DMP image was built in isolation.
- Production C codec passes ASan/UBSan in raw C3, DMP S3, dummy C3 and disabled
  AUX configurations. Covers field boundaries, signed extrema, clock/sequence
  wrap, immutable inputs/output on failure, fixed zero peer slots, inactive ADC
  slots and MTU 23/128/247/517 fragmentation round trips.
- C-generated golden packets in `test/fixtures/batch_v3.json` are decoded by
  the Dart production models, directly and after MTU23 fragment reassembly.
  Every sample/status field is checked, including signed errors and counters.
- Production sample queues pass ASan/UBSan with mutex shims, lifecycle/overflow
  and metadata boundaries, epoch fencing and concurrent producers/consumers.
  All 90,000 concurrent input records are accounted for as intact output or
  explicitly counted oldest-record overflow; the split varies by scheduling.
- Production sender functions pass deterministic interleaving regressions:
  transient enqueue failures retain identical bytes, unsubscribe/reconnect
  reject old jobs, an outstanding MTU change does not duplicate a batch, a
  successful final enqueue is not falsely counted as expired, a persistent
  enqueue failure expires/counts its batch exactly once, and all five
  streams share the six-job budget at MTU23.
- Acquisition regressions cover every accepted DMP FIFO quaternion, rollback
  of incomplete/error drains, existing ADC configurations and magnetometer
  recovery. Firmware configuration checks cover current/raw/DMP/ADC/dummy,
  one-channel, power-disabled and all-disabled variants.
- App tests cover malformed and absent/peer-only slots, sequence wrap,
  fragmentation expiry/order, legacy raw/status/AUX, chart sample spacing,
  CSV row count/timing/device/session/glove provenance, atomic rejection,
  reconnect state, controls, accessibility and Windows subscription behavior.

## Hardware checks

The USB sensor is a XIAO ESP32-C3, advertised as `XIAO-IMU-RAW-75F2`.
Raw IMU uses the physical ICM-20948/AK09916; AUX is configured as a three-channel
**dummy** source at 100 Hz. No electrical ADC accuracy validation is claimed.
The client is a macOS CoreBluetooth/Bleak harness, not the Flutter UI.

The final-image 60-second test requested 45/90/200/45 Hz for 15 seconds each at
MTU247. It received **3,632 raw samples in 963 notifications** and **5,870 AUX
samples in 1,174 notifications**, plus 235 status snapshots. All sequences were
continuous and all format/zero-slot/magnetic checks passed. Notification/sample
loss, FIFO and I2C counters remained zero. Four-second resubscription and
five-second reconnect checks also passed, with fresh records and zero gaps.
The checked-in parameterized harness passed an additional short run (two
seconds per requested rate), retaining the full-run evidence separately.

Exact counters and the flashed binary SHA256 are recorded in the
firmware repository under `tests/hardware/batch_v3_1.4.0_2026-11-16.json`.
The harness verifies raw/AUX/status frame sizes and zero-filled second slots,
magnetic validity, GATT snapshot reads, the standard one-byte battery value,
acknowledged command writes at requested 45/90/200/45 Hz, continuous sequence
ordering, unsubscribe/resubscribe, and disconnect/reconnect without stale data.

The first run exposed a mixed measured-rate metadata batch rejection. Queue
splitting now includes measured rate; its native regression prevents recurrence.
Independent sender review exposed MTU/completion and expiry/completion races;
both have deterministic regression coverage.

## Limits

Batching reduces notification count while the requested fixed empty glove slot
adds bytes. The hardware may acquire below the requested rate (especially
200 Hz); requested rate, measured acquisition rate and notification packet rate
must be assessed separately. Small MTUs add fragmentation overhead and cannot
provide the same throughput. Overflow/retry-expiry counters expose bounded loss.

Windows native/Chrome, Android, iOS, Linux, a physical DMP stream, electrical ADC
inputs and a hardware link constrained to MTU23 have not been verified in this
release. MTU23 behavior is covered by C/Dart and sender regressions. The results
do not establish a fix for all radio/environmental disconnect causes. Second
slot decoding/reservation is implemented; interglove transport is intentionally
outside this release.
