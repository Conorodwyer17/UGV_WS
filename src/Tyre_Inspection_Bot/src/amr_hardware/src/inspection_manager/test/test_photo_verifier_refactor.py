from inspection_manager.photo_verifier import PhotoVerifier


def test_photo_verifier_rejects_missing_file():
    v = PhotoVerifier()
    ok = v.verify(
        "/tmp/does_not_exist.png",
        {
            "vehicle_id": "v",
            "tire_id": "t",
            "timestamp": "x",
            "pose": {},
            "camera_intrinsics": {},
            "projection_overlap": 0.8,
        },
    )
    assert ok is False
