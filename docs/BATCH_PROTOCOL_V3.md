# Paired sensor batches, protocol v3 (firmware 1.4.0)

All integers are little endian. Every custom sensor notification uses v3. UUIDs
are unchanged. Slot 0 is the transmitting/local glove. Slot 1 is reserved for a
future second glove. Presence masks 1, 2 and 3 are valid for decoding. Every frame physically includes both slots; absent slot
metadata and sample bytes MUST be zero. A presence mask distinguishes an absent
glove from real zero measurements. This release transmits mask 0x01 only. There
is no master/slave discovery, pairing, relay or synchronization implementation.

## Sensor frames (raw=1, quaternion=2, AUX=3)

Common 28-byte header:

| Offset | Bytes | Field |
|---|---:|---|
| 0 | 1 | version=3 |
| 1 | 1 | kind: 1 raw, 2 quaternion, 3 AUX |
| 2 | 1 | present slots: bit0 local, bit1 reserved peer; other bits zero |
| 3 | 1 | sample row count, 1..8 |
| 4 | 12 | slot 0 metadata |
| 16 | 12 | slot 1 metadata (zero while absent) |

Each slot metadata is: base sequence u32 at +0, base device timestamp_ms u32 at
+4, measured acquisition rate_hz u16 at +8, format metadata u8 at +10, reserved
zero u8 at +11. Raw format metadata packs accel range in bits0..1 and gyro range
in bits2..3 (upper bits zero). AUX metadata is active channel count 1..8.
Quaternion metadata is zero. Metadata is constant within a batch.

Rows start at offset28; each row is slot0 then slot1. Every slot starts with
sequence_delta u8 (+0), timestamp_delta_ms u16 (+1), sample flags u8 (+3).
Reconstruct sequences/timestamps modulo 2^32 using that slot's bases. First-row
deltas are zero. Later sequence deltas strictly increase; time deltas are
nondecreasing. Encoding must split before a delta exceeds its field width or
metadata changes; it must never silently fabricate evenly spaced times.

| Kind | Slot bytes | Slot payload after 4-byte delta/flags prefix |
|---|---:|---|
| raw 1 | 24 | accel XYZ, gyro XYZ, mag XYZ, temperature: ten i16 |
| quaternion 2 | 22 | q0/q1/q2/q3 Q30: four i32; accuracy u16 |
| AUX 3 | 20 | eight i16 ADC counts; unused channels zero |

Raw and quaternion flags retain their existing full v1 bit definitions. AUX
flag bit0 means valid; all other bits zero. Slot 1 has the same fixed width as
slot0 even when absent. Exact frame length is 28 + rows * 2 * slot_bytes.
Malformed frames are rejected atomically before emitting any sample.

## Status frame (kind=4)

Status is a current snapshot, not a high-rate acquisition stream. Header is
[3,4,presence_mask,1], followed by two fixed 52-byte status slots (108 total).
Absent slot1 is zero. Slot fields:

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | uptime_ms |
| 4 | u16 | requested IMU acquisition rate_hz |
| 6 | u16 | actual sensor rate_hz (raw when enabled, otherwise DMP) |
| 8 | u16 | IMU notification packet rate_hz (not sample rate) |
| 10 | u16 | battery_mV |
| 12 | u16 | ATT MTU |
| 14 | u8 | board ID: C3=1, S3=2 |
| 15 | u8 | IMU state |
| 16 | u8 | battery_percent |
| 17 | u8 | existing device flags (connected/advertising/pairing/etc.) |
| 18 | 3*u8 | firmware major/minor/patch |
| 21 | u8 | PHY (0 unknown) |
| 22 | u16 | feature flags: bit0 batched, bit1 fixed peer slot; value3 |
| 24 | u32 | free_heap_bytes |
| 28 | u32 | permanently dropped notifications |
| 32 | u32 | FIFO recoveries/overflows counter (existing meaning) |
| 36 | u32 | I2C errors |
| 40 | u32 | discarded sensor samples due to buffer/retry expiry |
| 44 | u32 | raw samples acquired |
| 48 | i32 | last_error |

Standard Bluetooth Battery Level (0x2A19) remains its specified one-byte value;
battery measurements are also present in the paired v3 status snapshot.
Firmware 1.4.1 adds an ADC rate control write `[0x08, rate_lo, rate_hi]` to
the existing Control characteristic. The rate is in Hz, valid from 1 to 200
when AUX ADC is enabled. This is independent of the IMU rate command (`0x01`).
The current RTOS tick rate and acquisition workload bound the measured rate,
which is included in each AUX batch. ADC rate returns to the configured default
on disconnect. Existing commands and packet formats remain compatible.

## MTU and fragmentation

Preferred ATT MTU is 247. Characteristic value capacity is min(MTU-3,512).
Batches use the largest whole row count fitting that capacity, capped at8,
and flush after at most80ms so low acquisition rates remain responsive.
Raw frames carry 2 rows at MTU128 (124 bytes) or4 rows at MTU247 (220 bytes).
Quaternion carries 2/4 rows; AUX carries2/5 rows respectively.

If even a single-row frame does not fit (e.g. ATT MTU23), transmit ordered
application fragments, retaining the same complete frame layout:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | marker0xF3 |
| 1 | u8 | original frame kind |
| 2 | u16 | frame ID, wrapping |
| 4 | u8 | fragment index, starting0 |
| 5 | u8 | fragment count |
| 6 | u16 | total unfragmented frame length, <=512 |
| 8 | bytes | frame bytes |

Nonfinal fragments have equal payload lengths; final payload is positive and
no larger. Reassembly is bounded to one frame per stream, expires after1s, and
resets at disconnect/unsubscribe/new frame0. Reject missing/out-of-order parts,
changed metadata, oversized frames and length mismatches. A partially received
frame never emits samples. Small-MTU links cannot provide the same throughput
as negotiated larger-MTU links; bounded overflow counters make losses visible.
GATT reads return one complete v3 frame (long reads are handled by ATT).

## Sampling and backpressure

Capture accepted acquisition records in bounded queues, rather than sampling
only latest caches from the notification task. Keep latest caches for reads.
Queue depth64 per stream, oldest unread sample discarded on overflow with an
explicit sample-drop counter. Send bounded work fairly across enabled streams.
Retain a prepared frame/fragment on transient enqueue failure; advance only
when NimBLE accepts it. Expire a pending batch after500ms with counters. Clear
queues/pending fragments at connection and subscription boundaries so old data
is not replayed into another session. Never hold sensor locks while sending.

Firmware 1.4.1 reserves space in the notification mbuf pool for GATT traffic.
The serial diagnostic fields `tx_drop`, `queue_drop`, and `raw_read_fail`
distinguish rejected transmissions, discarded buffered samples, and failed
IMU acquisitions. Acquisition can run independently of BLE. A degraded IMU
keeps BLE, status, and AUX available. The IMU uses the stock 1.4.0 startup
sequence and clock setting. Runtime I2C recovery arms after the first valid
hardware sample. A runtime timeout schedules the first attempt after 50 ms,
and failed attempts back off to five seconds. Recovery logs the original
transfer error, register, and SCL/SDA levels. It first clears the bus without
resetting the IMU. If identity or power-state checks fail, a later attempt
reapplies the stock sensor configuration. After an initial startup failure,
the acquisition task makes up to three retries one second apart at an I²C
clock capped at 100 kHz. Runtime recovery still requires a valid hardware sample.

App ingestion expands every batch entry into history, analysis and CSV using
the original device timestamp and sequence. Preserve true callback arrival
separately from a derived chart time; do not collapse a batch onto one chart X
coordinate. Glove IDs are retained in decoded records/export, and reserved
slots do not create synthetic readings or mix into the local analysis pipeline.
