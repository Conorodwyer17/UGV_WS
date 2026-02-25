# Acceptance Criteria (2026-02-25)

## Binary pass conditions
1. Vehicle detection logged (e.g., `DETECTION_EVENT` or `vehicle_detected` JSONL entry).
2. Detection triggers goal computation (`GOAL_COMPUTED`).
3. Goal transforms correctly in map frame (goal frame = `map` and TF valid).
4. Robot drives toward vehicle without unnecessary spin (cmd_vel forward motion observed).
5. Robot approaches to configured photo distance (or `photo_trigger_distance` disabled).
6. Photo captured and saved (capture_result `SUCCESS`, file exists).
7. Mission reports SUCCESS state (`DONE` with report showing expected tires captured).
8. Logs preserved after reboot (mission JSONL + journald entries persist).

## Current status
- **FAIL** (not executed): No acceptance run performed after fixes; no rosbag or capture evidence available.
