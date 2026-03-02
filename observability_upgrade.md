# Observability Upgrade

Mission summary JSON, tire-level metrics, segmentation metrics, optional debug topic.

---

## 1. Current State

| Metric | Location | Format |
|--------|----------|--------|
| Mission report | _mission_report dict | JSON in mission log |
| tires_captured | mission_report | int |
| tires_skipped | mission_report | list of dicts |
| total_tires_expected | mission_report | int |
| Mission log | _mission_log_append | List of events |
| nav_result | mission_log | status, distance_to_goal, etc. |

---

## 2. Proposed Additions

### 2.1 Mission Summary JSON (Enriched)

Add to mission_report at mission end:

| Field | Type | Purpose |
|-------|------|---------|
| mission_duration_s | float | Total mission time |
| approach_time_per_tire_s | list | Per-tire approach duration |
| retries_per_tire | list | Per-tire nav retry count |
| final_distance_error_m | list | Distance to goal at capture |

### 2.2 Tire-Level Metrics

| Metric | Source | Notes |
|--------|--------|-------|
| approach_time | _on_box_result timestamp | Time from dispatch to success |
| retries | _nav_retry_count at capture | Per-tire |
| final_distance_error | _distance_to_current_goal at photo | Before trigger |

### 2.3 Segmentation Metrics

| Metric | Source | Notes |
|--------|--------|-------|
| cluster_count | segmentation_processor | Log per callback |

### 2.4 Nav2 Metrics

| Metric | Source | Notes |
|--------|--------|-------|
| planner time | Nav2 feedback | If available |
| controller time | Nav2 feedback | If available |

**Implementation:** Nav2 action feedback may contain timing; inspect NavigateToPose feedback structure. Optional; not required for determinism.

---

## 3. Optional Debug Topic

```
/inspection_debug/mission_snapshot
```

**Type:** std_msgs/String (JSON)

**Content:** mission_id, state, current_goal, tires_captured, tires_skipped, last_event

**Publish rate:** On state change or 1 Hz when active

**Rationale:** Helps diagnose repeatability without full mission log dump.

---

## 4. Implementation Priority

| Item | Priority | Effort |
|------|----------|--------|
| mission_duration_s in report | High | Low |
| approach_time_per_tire | Medium | Medium |
| retries_per_tire | Medium | Low |
| final_distance_error | Medium | Low |
| /inspection_debug/mission_snapshot | Low | Medium |
| cluster_count log | Low | Low |

---

## 5. Minimal Additions (Recommended)

- Add `mission_duration_s` to mission_report (compute from start/end timestamps).
- Add `tire_metrics` list to mission_report: each entry has tire_position, approach_time_s, retries, final_distance_m.
- Do not overcomplicate; only add metrics that help diagnose repeatability.
