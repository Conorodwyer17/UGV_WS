# Nav2 Deep Best Practice Alignment Report

Review against official Nav2 documentation and config examples.

---

## 1. Documentation Alignment

| Source | Finding | Status |
|--------|---------|--------|
| Nav2 costmap_2d | origin_x/y for rolling window | Done: 2.5, 2.5 |
| Nav2 controller | transform_tolerance 0.5-1.0 s | Done: 0.5 |
| Nav2 #4299 | local costmap global_frame: odom | Done |
| PATH_FORWARD | inflation 0.6 local, 0.35 global | Done |
| best_practices_matrix | controller_frequency 10 Hz when CPU contended | Optional: 20 Hz; consider 10 for Jetson |

---

## 2. Controller: DWB vs Regulated Pure Pursuit

Keep DWB + RotationShim. Regulated Pure Pursuit would require replacing primary controller; DWB already tuned. No switch justified.

---

## 3. Inflation vs Vehicle Geometry

inflation_radius 0.6 local, 0.35 global. Both above robot half-width. No change.

---

## 4. Rolling Window

width/height 5 m, origin 2.5, 2.5. Matches ROS Answers 416578. No change.

---

## 5. Behavior Tree vs FollowWaypoints

Both supported. use_follow_waypoints: false for production until batch validated. BT removes spin from recovery.

---

## 6. Summary

Nav2 config aligned. No controller or costmap changes required. Optional: controller_frequency 10 in PRODUCTION_CONFIG for Jetson.
