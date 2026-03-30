# Planning test with stub motor (no physical motion)

Use this to **validate Nav2 goals and global/local plans in RViz** without the base moving (`/cmd_vel` discarded). Helps separate **planning correctness** from **control / CPU / TF latency** during real motion.

## What the mission log showed (archive: `logs/mission_latest.jsonl`)

From the most recent captured run (vehicle + tyres detected, real motion intended):

- **Goal placement:** A goal was computed and sent in `map` with **`nav_command_sent`** (`offset` **0.7 m**, `yaw_deg` **135°**) toward the **nearest corner** phase (`mission_plan_dispatch` → `approach_nearest_corner`). The log marks **`planned_tire_vehicle_center_goal`: true** for that goal—i.e. first move is **approach geometry**, not necessarily the final tyre standoff yet.
- **Planner / controller:** **`nav_command_sent`** indicates Nav2 accepted a goal; the JSONL does **not** include planner internals. **`progress_stall`** fired in **`INSPECT_TIRE`** after **5 s** with **`distance_m` ~0.03 m**—almost **no progress** toward the goal (consistent with **slip, stuck, control loop issues, or heavy load**, not necessarily “no plan”).
- **Multiple vehicle tracks:** **`vehicle_detected`** with **`vehicle_id`: 2** and a new **`tire_plan`** shortly after commit suggests **perception / ID churn** (map or semantic box moved), which can confuse mission logic if not gated.
- **Photo capture:** No **`VERIFY_CAPTURE`**, **`capture_photo`**, or **`Centroid`** handoff lines appear in this short log—the mission did **not** complete a tyre photo on this run.

**Interpretation:** The stack **did** send a navigation goal; failure to “reach the tyre” and take a photo is **not** proven to be “wrong goal maths” from this file alone. It is **consistent with** (1) **little or no executed motion** (`progress_stall`), (2) **system overload** (TF extrapolation / control warnings you saw on console), and/or (3) **state / vehicle-ID instability**. A **stub-motor** run lets you confirm **/plan** and **/local_plan** in RViz without draining the battery or fighting latency.

**Code update (March 2026):** When **`use_tyre_3d_positions`** is true, goals from **`/tyre_3d_positions`** now use a **robot–tyre standoff** (offset along **robot → tyre**), not **vehicle_center → tyre** (which was wrong for tiny default vehicle boxes). Refinement and capture proximity radii were widened; default **`tire_offset`** is **0.3 m**. **`full_bringup`** defaults **`use_tyre_3d_positions:=true`** and **`inspection_delay_s:=90`** (override as needed).

---

## Jetson: full stack, same CPU tyre settings, stub motor

Matches **`MISSION_PROFILE=mission_dedicated_cpu`** except **`sim_no_move:=true`** (stub motor node eats `/cmd_vel`).

```bash
cd ~/ugv_ws && source install/setup.bash
export ROS_DOMAIN_ID=0   # match laptop
MISSION_PROFILE=stable_viz ./scripts/start_mission.sh --no-verify
```

Ensure **`tyre_detection_project/best.onnx`** exists (IMGSZ **480** with default `wheel_imgsz`):

```bash
# if needed
MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh
```

Wait until the mission leaves **IDLE** (TF + Nav2 ready per `PRODUCTION_CONFIG`). Then place the vehicle in view as for a real run.

**Alternative (longer inspection delay, packaged RViz):**  
`ros2 launch ugv_bringup demo_full_visualization.launch.py` — also uses stub motor and CPU tyre path by default; good for **pipeline demos**; timing differs from `start_mission.sh`.

---

## Laptop: RViz

Same Wi‑Fi, same **`ROS_DOMAIN_ID`**, Cyclone DDS:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash   # if workspace installed
~/ugv_ws/scripts/monitor_mission.sh
```

**Fixed frame:** `map` (if TF looks wrong, try **`slamware_map`** in Global Options).

**What to look for**

- Red **global plan** topic **`/plan`**, blue **local** **`/local_plan`** when a goal is active.
- Tyre overlay **`/ultralytics_tire/segmentation/image`**, **`/tyre_markers`**, **`/aurora_semantic/vehicle_markers`**.

If **plans look reasonable** to the tyre standoff but real motion failed → prioritize **load reduction** and **TF stability** before blaming planner maths.

---

## Reduce load for the next real-motion attempt

Apply in order (re-export ONNX if you change **`wheel_imgsz`**):

| Tuning | Example | Notes |
|--------|---------|--------|
| Tyre inference rate | `inference_interval_s:=1.0` | Less CPU; fewer overlays |
| Input size | `wheel_imgsz:=320` | Requires **`best.onnx` at 320** (`export_onnx.sh` with `IMGSZ=320`) |
| Depth cloud | `depth_registered_publish_hz:=2.0` | Already typical; keep |
| Costmap | `costmap_resolution:=0.10` | Already typical; keep |
| Monitor | `tegrastats --interval 500 --logfile ~/ugv_ws/benchmarks/tegra_motion.log &` | RAM/CPU |

Example real-motion launch (after stub validation):

```bash
./scripts/start_mission.sh --no-verify \
  inference_interval_s:=1.0 \
  wheel_imgsz:=480
```

(For `wheel_imgsz:=320`, export ONNX at 320 first.)

---

## Checklist: real motion (after stub plan looks good)

- [ ] **`best.onnx`** matches **`wheel_imgsz`**
- [ ] Battery sufficient; e-stop accessible
- [ ] Robot **well inside** mapped area (~2 m from vehicle as intended)
- [ ] **`tegrastats`** logging
- [ ] Same **`ROS_DOMAIN_ID`** if using laptop RViz
- [ ] If TF / control warnings return, stop and capture **`~/.ros/log/**` and full **`mission_latest.jsonl`**

---

## If no plan appears in RViz

- Confirm **Nav2 lifecycle** active (`ros2 lifecycle nodes`).
- **`ros2 topic echo /plan --once`** while goal pending.
- Map size / **goal in bounds** (goal off costmap → planner failure).
- **`ros2 topic echo /tf`** — `map` ↔ `base_link` stable.

For deeper forensics, provide **full console log** from the failed run and **`~/.ros/log/<date>/`** planner/controller excerpts.
