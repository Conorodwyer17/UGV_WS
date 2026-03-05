# Fresh Installation Guide — Jetson Orin Nano (16 GB)

This document describes how to set up a new NVIDIA Jetson Orin Nano Developer Kit (16 GB) from scratch for the autonomous tyre inspection robot. For the 8 GB Jetson, use `use_cpu_inference:=true` or follow the same steps; CPU inference is the fallback for 8 GB.

---

## 1. Hardware Requirements

- **NVIDIA Jetson Orin Nano Developer Kit** (16 GB recommended; 8 GB supported with CPU inference fallback)
- **SLAMTEC Aurora** (Ethernet or Wi‑Fi)
- **Waveshare UGV Rover** with ESP32 motor controller, UART connection
- **Power supply** (e.g. 12 V for the rover base; Jetson powered separately or from base)
- **Linux host computer** (Ubuntu 20.04 or 22.04, x86) for flashing JetPack — required for first-time bootloader update

---

## 2. Flash JetPack 6.0

**Important:** Fresh Orin Nano units ship with outdated factory firmware. You must update the QSPI bootloaders first using SDK Manager before the board will boot correctly.

### 2.1 Prerequisites

- Linux host (Ubuntu 20.04 or 22.04, x86)
- USB‑C cable to connect Jetson to host
- Put Jetson in **Force Recovery Mode**: disconnect power, place jumper on REC/GND pins, reconnect power

### 2.2 Install via SDK Manager

1. Download [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager) from the Jetson download page.
2. Install SDK Manager on your Linux host.
3. Connect the Jetson via USB‑C and put it in Force Recovery Mode.
4. Run SDK Manager and select:
   - **JetPack 6.0** (or latest L4T R36.x)
   - **Jetson Orin Nano** (16 GB or 8 GB)
   - Target: SD card or NVMe SSD (NVMe recommended for faster I/O)
5. Flash the QSPI bootloaders and OS. This may take 30–60 minutes.
6. After flashing, remove the jumper, connect display/keyboard/mouse, and boot into the new Ubuntu image.

### 2.3 Post‑flash (on the Jetson)

```bash
sudo apt update && sudo apt upgrade -y
sudo systemctl set-default graphical.target   # if you want a desktop
```

---

## 3. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (desktop + Nav2)
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-nav2-bringup ros-humble-robot-localization

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Install Python Dependencies

### 4.1 PyTorch (with CUDA for Jetson)

For JetPack 6.0, use the NVIDIA‑provided PyTorch wheel:

```bash
# Install pip if needed
sudo apt install -y python3-pip

# Install PyTorch with CUDA (check NVIDIA Jetson download page for latest wheel)
# Example for JetPack 6.0 / L4T R36.x:
pip3 install --user torch torchvision

# Or for Jetson-specific wheel (from NVIDIA):
# wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.1.0a0+dfd0a5c-cp310-cp310-linux_aarch64.whl
# pip3 install --user torch-2.1.0a0+dfd0a5c-cp310-cp310-linux_aarch64.whl
```

Verify CUDA:

```bash
python3 -c "import torch; print('CUDA:', torch.cuda.is_available()); print('Device:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')"
```

### 4.2 Ultralytics, ONNX Runtime, and Other Dependencies

```bash
pip3 install --user ultralytics onnxruntime-gpu
```

For CPU inference fallback (8 GB Jetson only):

```bash
pip3 install --user onnxruntime
```

---

## 5. Clone Repository and Build

```bash
cd ~
git clone <your-repo-url> ugv_ws
cd ugv_ws
```

### 5.1 Install Aurora ROS 2 SDK

The SLAMTEC Aurora ROS 2 SDK must be in `src/`. If it is not included in the repo:

1. Download from [SLAMTEC Aurora ROS 2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/).
2. Extract into `src/aurora_ros2_sdk_linux/` (or equivalent path used by the build).

### 5.2 Install Workspace Dependencies

```bash
cd ~/ugv_ws
pip3 install --user -r src/Tyre_Inspection_Bot/requirements.txt
rosdep install --from-paths src --ignore-src -r -y
```

### 5.3 Build

```bash
cd ~/ugv_ws
colcon build --symlink-install
source install/setup.bash
```

### 5.4 Add to ~/.bashrc

```bash
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Aurora SDK library path (Jetson = aarch64)
echo 'export LD_LIBRARY_PATH=~/ugv_ws/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH' >> ~/.bashrc
```

---

## 6. Model Files

Place the tyre detection model in the workspace:

```bash
# Copy best_fallback.pt to:
cp /path/to/best_fallback.pt ~/ugv_ws/src/Tyre_Inspection_Bot/
```

### 6.1 Export TensorRT Engine (16 GB Jetson)

```bash
cd ~/ugv_ws && bash scripts/export_tensorrt.sh
```

Default: `imgsz=640`, `workspace=8` GB. For 8 GB Jetson, use:

```bash
IMGSZ=320 WORKSPACE=4 ./scripts/export_tensorrt.sh
```

---

## 7. Network and UART Configuration

### 7.1 Aurora (Ethernet)

- Connect Aurora via Ethernet. In access‑point mode, Aurora is typically at `192.168.11.1`.
- Configure the Jetson’s Ethernet interface to be on the same subnet (e.g. static IP `192.168.11.100`).
- Test: `ping -c 1 192.168.11.1`

### 7.2 UART (Motor Driver)

- The Waveshare UGV uses `/dev/ttyTHS1` (or similar) for the ESP32.
- Add user to `dialout` group: `sudo usermod -aG dialout $USER`
- Verify: `ls -l /dev/ttyTHS1` (or your UART device)

---

## 8. Differences: 16 GB vs 8 GB Jetson

| Setting | 16 GB Jetson | 8 GB Jetson |
|---------|--------------|-------------|
| `use_cpu_inference` | `false` (default) | `true` |
| TensorRT engine | Recommended | Optional (may OOM) |
| `workspace` (export) | 8 GB | 4 GB |
| `imgsz` (export) | 640 | 320 |
| Inference rate | 10 Hz | 5 Hz (CPU) |

---

## 9. Quick Verification

After setup:

```bash
source ~/ugv_ws/install/setup.bash
python3 ~/ugv_ws/scripts/verify_system.py --skip-ros
```

Then run a simulated mission (no Aurora or vehicle required):

```bash
./scripts/start_mission.sh --no-verify
# Use use_mock:=true sim_tyre_detections:=true if launching manually
```

---

## 10. Optional: One‑Click Setup Script

For a faster setup, use `scripts/setup_jetson.sh` (idempotent, safe to re‑run):

```bash
cd ~/ugv_ws && bash scripts/setup_jetson.sh
```

See the script for what it installs and any manual steps it cannot automate.
