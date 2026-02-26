# Verification Checklist (Dry-Run + Field)
#
# Record results next to each step.

## Dry-run (no motion)
1. Launch full stack with dry_run:
   - `bash scripts/mission_launch.sh dry_run:=true`
2. Confirm readiness:
   - `bash scripts/mission_readiness_check.sh`
3. Verify perception topics:
   - `ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once`
   - `ros2 topic echo /darknet_ros_3d/bounding_boxes --once`
4. Verify mission flow in logs:
   - `logs/mission_latest.jsonl` contains `vehicle_detected`, `goal_computed`, `nav_command_sent` (dry-run), `FACE_TIRE` transitions, and `photo_triggered`.

Result: ☐ PASS ☐ FAIL  
Notes:

## Field run (real motion)
1. Launch full stack (no dry_run):
   - `bash scripts/mission_launch.sh`
2. Confirm readiness:
   - `bash scripts/mission_readiness_check.sh` (ensure nav_permitted true)
3. Mission completes full cycle:
   - detect → approach vehicle → detect tire → approach tire → face tire → capture photo (repeat ×4) → DONE.
4. Capture validation:
   - `logs/mission_report_latest.json` shows expected tires captured and success.
   - `~/ugv_ws/tire_inspection_photos` contains new images.

Result: ☐ PASS ☐ FAIL  
Notes:
