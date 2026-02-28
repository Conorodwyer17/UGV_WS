# ESP32 Serial/Odometry Bridge Metadata

- capture_date_utc: `2026-02-28`
- probed_ports: `research/data/esp32_odometry/esp32_probe_20260228_005020.json`
- connected_port: `/dev/ttyTHS1`
- baud: `115200`
- raw_serial_log: `research/data/esp32_odometry/esp32_serial_20260228_005020.log`

## Observed Telemetry Format

- repeated JSON packets with `T=1001`
- fields observed:
  - drive/odom: `L`, `R`, `odl`, `odr`
  - IMU: `ax`, `ay`, `az`, `gx`, `gy`, `gz`
  - magnetometer: `mx`, `my`, `mz`
  - battery: `v`

## Bridge Implementation

- bridge node path: `nodes/esp32_bridge/esp32_serial_bridge_node.py`
- outputs:
  - `/esp32/telemetry_json`
  - `/esp32/encoders_raw`
  - `/odom`
- startup/reference doc: `nodes/esp32_bridge/README.md`
