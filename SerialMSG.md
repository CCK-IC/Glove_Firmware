# Firmware serial diagnostics

The periodic `diag` log reports acquisition rates separately from BLE
notification rates. All rate fields are per second.

| Field | Meaning |
|---|---|
| `board`, `fw`, `idf` | Board name, running firmware version, and ESP-IDF version |
| `ble`, `mtu` | BLE connection state and negotiated ATT MTU |
| `imu`, `dmp` | IMU state code and DMP enabled flag |
| `dmp_notify`, `raw_notify`, `aux_notify` | Notification subscription flags |
| `imu_req_hz` | Requested IMU acquisition rate |
| `dmp_hz`, `raw_hz` | Measured DMP and raw IMU sample rates |
| `imu_notify_hz` | Successful raw and DMP notification submissions per second, combined |
| `adc_req_hz`, `adc_hz` | Requested and measured ADC sample rates |
| `adc_notify_hz` | Successful AUX notification submissions per second |
| `bat` | Battery voltage in mV and charge percentage |
| `samples`, `raw_samples`, `adc_samples` | Cumulative acquired DMP, raw IMU, and ADC samples |
| `tx_drop`, `queue_drop`, `raw_read_fail` | Dropped notifications, discarded queued samples, and raw IMU read failures |
| `i2c`, `fifo`, `heap`, `led` | I²C errors, FIFO overflows, free heap bytes, and active LED pattern |

Each BLE submission may contain a batch of samples or an individual fragment.
The status characteristic also reports the IMU and ADC notification rates.
[Status packet layout](docs/BATCH_PROTOCOL_V3.md#status-frame-kind4)
