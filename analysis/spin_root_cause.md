# 180° Spin Analysis (2026-02-25)

## What the code commands during search
Rotation during search is **explicitly commanded** by `_dispatch_rotation_goal`, using `rotation_angle` and a small forward offset to force Nav2 to rotate.

```86:88:/home/conor/.cursor/worktrees/ugv_ws__SSH__jetson_Thesis_WIFI_/rxd/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py
        self.declare_parameter("rotation_angle", 0.785)  # 45 degrees in radians
        self.declare_parameter("max_rotation_attempts", 8)  # 8 * 45 = 360 degrees
        self.declare_parameter("rotation_position_offset", 0.1)  # Small forward offset to force Nav2 execution
```

```1706:1783:/home/conor/.cursor/worktrees/ugv_ws__SSH__jetson_Thesis_WIFI_/rxd/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py
    def _dispatch_rotation_goal(self, is_vehicle: bool = True, is_search: bool = False):
        """Dispatch a rotation goal to turn in place by rotation_angle.
        ...
        """
        ...
        rotation_angle = self.get_parameter("rotation_angle").value
        new_yaw = current_yaw + rotation_angle
        ...
        offset = self.get_parameter("rotation_position_offset").value
        ...
        self.get_logger().info(
            f"Rotation goal (attempt {self.rotation_attempts}):\n"
            f"  Current yaw: {current_yaw_deg:.2f}° ({current_yaw:.3f} rad)\n"
            f"  Target yaw: {new_yaw_deg:.2f}° ({new_yaw:.3f} rad)\n"
            f"  Yaw difference: {yaw_diff_deg:.2f}° ({yaw_diff:.3f} rad)\n"
            f"  Position offset: {offset:.3f}m\n"
            f"  Goal position: ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})"
        )
```

## Observed rotation in 2026-02-25 logs
The inspection manager logs show **45° search rotations** (not 180°). Example from the 12:37 run:
```
9:16:/home/conor/.ros/log/python3_3266_1772023057280.log
[INFO] [1772023064.715884264] [inspection_manager]: No vehicle detected after 6.0s. Rotating to search (attempt 1/8).
[INFO] [1772023064.718311944] [inspection_manager]: Rotation goal (attempt 1):
  Current yaw: -175.74° (-3.067 rad)
  Target yaw: -130.77° (-2.282 rad)
  Yaw difference: 44.98° (0.785 rad)
  Position offset: 0.100m
  Goal position: (0.368, -0.056)
```
Nav2 **spin behavior actually executes** in the same window:
```
23:27:/home/conor/.ros/log/behavior_server_3097_1772023011378.log
[INFO] [1772023067.562656648] [behavior_server]: Running spin
[INFO] [1772023067.563463304] [behavior_server]: Turning 1.57 for spin behavior.
[WARN] [1772023069.464053736] [behavior_server]: Collision Ahead - Exiting Spin
[WARN] [1772023069.464165928] [behavior_server]: spin failed
```

## Likely causes of a 180° spin (if observed visually)
1. **Nav2 Spin recovery**: behavior server has a Spin plugin **and it is executed** (see log above); if a goal is behind or unreachable, recovery can rotate in place.
```
205:208:/home/conor/.cursor/worktrees/ugv_ws__SSH__jetson_Thesis_WIFI_/rxd/analysis/journal_ugv_mission_2026-02-25_1240-1310.log
2026-02-25T13:01:25+0000 conor-desktop ugv_mission[3525]: [behavior_server-20] [INFO] [1772024485.729742776] [behavior_server]: Creating behavior plugin spin of type nav2_behaviors/Spin
2026-02-25T13:01:25+0000 conor-desktop ugv_mission[3525]: [behavior_server-20] [INFO] [1772024485.735268666] [behavior_server]: Configuring spin
```
2. **Goal orientation mismatch**: a goal orientation computed in `slamware_map` but executed in `map` could cause an apparent 180° turn if frames are misaligned or TF is stale.
3. **Rotation shim** in Nav2 controller: if a goal requires facing opposite direction, the rotation shim can command an in-place turn before linear motion.

## Gaps / Missing Evidence
- No `/cmd_vel` or Nav2 feedback logs are persisted for the “spin” moment.
- The log that showed bounding box overlays is missing from this host.

## Required verification
- Capture `/cmd_vel`, `/tf`, and Nav2 feedback during the next reproduction to confirm whether the spin is commanded by inspection_manager (rotation search) or by Nav2 recovery.

## Confidence
**Low** for the 180° root cause: the accessible logs show 45° rotation goals and do not contain `/cmd_vel` evidence. The spin recovery plugin is configured and **executed**, but it is not proven to have caused the 180° turn in the user’s described run.

