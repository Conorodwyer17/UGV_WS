from inspection_manager.approach_planner import ApproachPlanner


def test_approach_planner_returns_goal():
    planner = ApproachPlanner()
    goal = planner.compute_goal({"side": "left", "position": {"x": 1.2, "y": 0.4}})
    assert goal["frame_id"] == "map"
    assert "x" in goal and "y" in goal

