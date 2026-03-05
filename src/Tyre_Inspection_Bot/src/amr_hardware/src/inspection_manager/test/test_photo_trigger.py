from inspection_manager.utils import should_trigger_photo, distance_point_to_aabb_2d


def test_distance_point_to_aabb_2d_outside():
    # Point (0, 0) to box [1, 2] x [1, 2]: nearest point (1, 1), dist = sqrt(2)
    assert abs(distance_point_to_aabb_2d(0, 0, 1, 2, 1, 2) - 1.414) < 0.01


def test_distance_point_to_aabb_2d_inside():
    # Point (1.5, 1.5) inside box [1, 2] x [1, 2]: nearest point (1.5, 1.5), dist = 0
    assert distance_point_to_aabb_2d(1.5, 1.5, 1, 2, 1, 2) == 0.0


def test_distance_point_to_aabb_2d_on_edge():
    # Point (0.5, 1) to box [0, 1] x [0, 1]: nearest (0.5, 1), dist = 0 (on edge)
    assert distance_point_to_aabb_2d(0.5, 1.0, 0, 1, 0, 1) == 0.0


def test_photo_trigger_within_distance():
    assert should_trigger_photo(0.2, 0.5) is True


def test_photo_trigger_blocks_when_far():
    assert should_trigger_photo(0.6, 0.5) is False


def test_photo_trigger_disabled_threshold():
    assert should_trigger_photo(1.5, 0.0) is True


def test_photo_trigger_unknown_distance():
    # When threshold > 0, unknown distance blocks capture (avoid photo from wrong position)
    assert should_trigger_photo(None, 0.5) is False
