# Stress Test Results

Stress test harness for repeated mission execution. Goal: 100% success under mild perturbation.

---

## 1. Harness Location

```
sim/stress_test_harness.py
```

Usage:
```bash
# Run 5 iterations (default)
python3 sim/stress_test_harness.py

# Run 10 iterations
python3 sim/stress_test_harness.py --iterations 10
```

---

## 2. Prerequisites

- Full system running: Aurora, Nav2, perception, inspection_manager (or follow_waypoints + photo_capture)
- Vehicle with tyres in view

---

## 3. Current Test Scope

| Test | Description |
|------|-------------|
| FollowWaypoints | 4 poses per mission |
| Success criteria | 0 missed waypoints, >= 4 photos |

---

## 4. Perturbation Modes (Future)

| Mode | Description | Status |
|------|-------------|--------|
| Repeated missions | N missions in loop | Implemented |
| Randomize vehicle pose | Slight pose variation |
| Detection jitter | Inject noise to bbox |
| Depth delay | Simulate 300 ms dropout |

Perturbation modes require simulation or mocking. Current harness uses real system.

---

## 5. Results Template

| Run | Date | Iterations | Success | Rate |
|-----|------|------------|---------|------|
| 1 | — | 5 | — | — |

---

## 6. Target

Mission success rate → 100% under mild perturbation. Document results after each run.
