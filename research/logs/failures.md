# Failure Records and Remediation

## 2026-02-28 Local Inventory

1. Command: `ros2 --version`
- Result: failed (`ros2: error: unrecognized arguments: --version`).
- Cause: ROS 2 CLI variant in this environment does not expose `--version`.
- Remediation: use `ros2 doctor --report` or package version queries (`dpkg -l | rg ros-humble-ros2cli`) for version traceability.

2. Command: `jetson_release`
- Result: failed (`command not found`).
- Cause: `jetson-stats` / `jetson_release` utility not installed in current image.
- Remediation: use fallback telemetry (`nvidia-smi`, `/etc/nv_tegra_release`, `uname -a`) and optionally install `jetson-stats`.
