from inspection_manager.utils import should_trigger_photo


def test_photo_trigger_within_distance():
    assert should_trigger_photo(0.2, 0.5) is True


def test_photo_trigger_blocks_when_far():
    assert should_trigger_photo(0.6, 0.5) is False


def test_photo_trigger_disabled_threshold():
    assert should_trigger_photo(1.5, 0.0) is True


def test_photo_trigger_unknown_distance():
    assert should_trigger_photo(None, 0.5) is True
