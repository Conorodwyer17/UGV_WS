from inspection_manager.state_persistence import MissionStatePersistence


def test_state_persistence_roundtrip(tmp_path):
    db_path = tmp_path / "missions.db"
    store = MissionStatePersistence(str(db_path))
    mission_id = store.start_mission("object_1", '{"max_retries":5}', False)
    store.add_tires(mission_id, ["axle1_left", "axle1_right"])
    store.set_tire_status(mission_id, "axle1_left", "completed", attempt_increment=1)
    snap = store.get_mission_state(mission_id)
    assert snap is not None
    assert snap["mission"]["mission_id"] == mission_id
    assert len(snap["tires"]) == 2
