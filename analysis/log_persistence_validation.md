# Log Persistence Validation (Plan + Evidence)

## Observed persistence (2026-02-25 run)
- ROS log directory contains only `launch.log` for the run.
- Runtime node output was captured in journald (`ugv_mission`), not per-node ROS logs.
- Mission JSONL logs are present under `/home/conor/ugv_ws/logs/archive/`.

## Risk
If the robot powers down during mission, buffered logs from RViz or AORA device may be lost if not flushed to disk.

## Validation procedure (must run on robot + AORA)
1. Start mission, ensure vehicle detection appears on-screen.
2. Confirm journald logging in real time:
   ```
   journalctl -u ugv_mission -f
   ```
3. Power off cleanly, reboot.
4. Confirm logs persisted:
   ```
   ls -la /home/conor/ugv_ws/logs/archive
   journalctl -u ugv_mission --since "2026-02-25 00:00" --until "2026-02-26 00:00" > /tmp/ugv_mission_2026-02-25.log
   ls -la /home/conor/.ros/log
   ```

## Pending evidence
No persistence test was executed in this pass; run the above and attach logs to this file.
