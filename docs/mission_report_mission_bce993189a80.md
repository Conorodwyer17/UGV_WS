# Mission Report `mission_bce993189a80`

- **Run type:** live validation attempt (unified `inspection_manager.launch.py`)
- **Date:** 2026-02-28
- **Result:** `DISCOVERY` (not complete)

## Metrics Summary

- `tyres_total`: 0
- `tyres_completed`: 0
- `success_rate`: 0.0
- metrics file: `research/logs/metrics/mission_bce993189a80.json`

## Evidence

- mission trace: `research/logs/missions/mission_bce993189a80.json`
- launch logs: `research/logs/inspection_unified_integration.log`

## Failure Analysis

- No vehicle/tyre detections were promoted into mission progression during this live run.
- Mission stayed in `DISCOVERY`, so no tyre planning, navigation, alignment, or capture occurred.

## Immediate Remediation

- Verify active publishers on configured detection topics before mission start.
- Gate mission start on positive world-model vehicle resolution.
- Ensure only one `/visual_servo/align` action server is running at a time.
