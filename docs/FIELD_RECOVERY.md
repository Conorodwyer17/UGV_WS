# Field Recovery Procedures

Procedures for recovering from common failures during field deployment.

## Reboot

```bash
sudo reboot
```

After reboot, if using systemd:
```bash
sudo systemctl start ugv_mission
```

Or manually:
```bash
cd ~/ugv_ws
source install/setup.bash
./scripts/start_mission.sh
```

## Access Logs

**Mission logs:** `~/ugv_ws/logs/`
- `mission_report_latest.json` – last mission outcome
- `mission_latest.jsonl` – per-tick state log

**ROS logs:** `~/.ros/log/` (latest build)

**Systemd (if used):**
```bash
journalctl -u ugv_mission -f
```

## Common Failure Modes

| Failure | Action |
|---------|--------|
| Aurora not connected | Check Ethernet; `ping 192.168.11.1`; power cycle Aurora |
| Motor not responding | Check `/dev/ttyTHS1`; user in `dialout` group |
| TF unavailable | Ensure Aurora is publishing; check `ros2 run tf2_ros tf2_echo slamware_map base_link` |
| Nav2 stuck | Cancel goal; check costmap in RViz; restart Nav2 lifecycle |
| Node crash | Check `~/.ros/log/` for stack trace; restart stack |

## Manual Control

If autonomy fails, use teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Ensure no other node is publishing to `/cmd_vel` (stop inspection_manager or use a mux).

## Rollback

If an update causes issues:
```bash
cd ~/ugv_ws
git checkout <previous-commit>
colcon build --symlink-install
source install/setup.bash
```
