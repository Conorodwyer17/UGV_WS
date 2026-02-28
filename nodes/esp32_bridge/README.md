# ESP32 Bridge (Minimal)

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
python3 ~/ugv_ws/nodes/esp32_bridge/esp32_serial_bridge_node.py
```

Optional parameters:

```bash
python3 ~/ugv_ws/nodes/esp32_bridge/esp32_serial_bridge_node.py \
  --ros-args \
  -p serial_port:=/dev/ttyTHS1 \
  -p baud_rate:=115200 \
  -p wheel_radius_m:=0.065 \
  -p wheel_base_m:=0.29 \
  -p ticks_per_rev:=2048.0
```

Published topics:

- `/esp32/telemetry_json` (`std_msgs/String`)
- `/esp32/encoders_raw` (`std_msgs/Int32MultiArray`)
- `/odom` (`nav_msgs/Odometry`)
