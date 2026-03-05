#!/bin/bash
# Idempotent setup script for Jetson Orin Nano (16 GB or 8 GB).
# Run from workspace root: cd ~/ugv_ws && bash scripts/setup_jetson.sh
# Safe to re-run; skips steps already done.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"

echo "=== Jetson Setup (idempotent) ==="
echo "Workspace: $UGV_WS"
echo ""

# 1. System updates (optional; can be slow)
if [[ "${SKIP_APT_UPDATE:-false}" != "true" ]]; then
  echo "[1/6] System update..."
  sudo apt update -qq
  sudo apt upgrade -y -qq || true
else
  echo "[1/6] Skipping apt update (SKIP_APT_UPDATE=true)"
fi

# 2. ROS 2 Humble (check if already installed)
if ! source /opt/ros/humble/setup.bash 2>/dev/null; then
  echo "[2/6] ROS 2 Humble not found. Installing..."
  sudo apt install -y -qq software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update -qq
  sudo apt install -y -qq ros-humble-desktop ros-humble-nav2-bringup ros-humble-robot-localization
else
  echo "[2/6] ROS 2 Humble already installed"
fi

# 3. Python dependencies
echo "[3/6] Python dependencies..."
pip3 install --user -q ultralytics onnxruntime 2>/dev/null || pip3 install --user ultralytics onnxruntime
if [[ -f "$UGV_WS/src/Tyre_Inspection_Bot/requirements.txt" ]]; then
  pip3 install --user -q -r "$UGV_WS/src/Tyre_Inspection_Bot/requirements.txt" 2>/dev/null || pip3 install --user -r "$UGV_WS/src/Tyre_Inspection_Bot/requirements.txt"
fi

# 4. Build workspace
echo "[4/6] Building workspace..."
if [[ -d "$UGV_WS/src" ]]; then
  cd "$UGV_WS" && colcon build --symlink-install
else
  echo "  src/ not found. Clone the repo first."
fi

# 5. Add to ~/.bashrc (idempotent)
echo "[5/6] Updating ~/.bashrc..."
BASHRC="$HOME/.bashrc"
if ! grep -q "ugv_ws/install/setup.bash" "$BASHRC" 2>/dev/null; then
  {
    echo ""
    echo "# ugv_ws setup"
    echo "source ~/ugv_ws/install/setup.bash 2>/dev/null || true"
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo 'export LD_LIBRARY_PATH=~/ugv_ws/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH'
  } >> "$BASHRC"
  echo "  Added ugv_ws setup to ~/.bashrc"
else
  echo "  ~/.bashrc already contains ugv_ws setup"
fi

# 6. UART permissions
echo "[6/6] UART permissions..."
if groups "$USER" | grep -q dialout; then
  echo "  User already in dialout group"
else
  sudo usermod -aG dialout "$USER"
  echo "  Added $USER to dialout. Log out and back in for it to take effect."
fi

echo ""
echo "=== Setup complete ==="
echo "Next steps:"
echo "  1. Place best_fallback.pt in $UGV_WS/src/Tyre_Inspection_Bot/"
echo "  2. (16 GB) Run: cd $UGV_WS && bash scripts/export_tensorrt.sh"
echo "  3. Connect Aurora (Ethernet) and verify: ping -c 1 192.168.11.1"
echo "  4. Run mission: ./scripts/start_mission.sh"
echo ""
echo "See SETUP.md for full installation guide."
