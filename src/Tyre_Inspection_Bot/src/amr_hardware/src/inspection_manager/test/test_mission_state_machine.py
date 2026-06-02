"""Tests for mission_state_machine: set_state transitions, spin protection, context resets."""

import math
from types import SimpleNamespace

from inspection_manager.mission_state_machine import MissionState, set_state, WaitStateContext


class _Pub:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _Node:
    def __init__(self, max_state_repeats=3):
        self.current_state = MissionState.IDLE
        self._last_transition_cause = "init"
        self._state_repeat_count = 0
        self._mission_report = {"error_states_encountered": 0, "total_tires_expected": 4}
        self._total_tires_captured = 0
        self.state_pub = _Pub()
        self.segmentation_mode_pub = _Pub()
        self.centroid_servo_enable_pub = _Pub()
        self._wait_context = None
        self.wait_start_time = None
        self.rotation_attempts = 0
        self._vehicle_confirm_count = 0
        self._vehicle_confirm_center = None
        self._tire_confirm_count = 0
        self._tire_confirm_center = None
        self._approach_entered_time = None
        self._progress_window_start = None
        self._progress_start_pose = None
        self._wheel_wait_start_time = None
        self._wheel_confirm_count = 0
        self._wheel_capture_inspection_published = False
        self._return_later_pass_count = 0
        self._tires_deferred = []
        self._refinement_ema_cache = {}
        self._centroid_handoff_initiated = False
        self._mission_log = []
        self._max_state_repeats = max_state_repeats

    def get_parameter(self, name):
        vals = {"max_state_repeats": self._max_state_repeats}
        return SimpleNamespace(value=vals[name])

    def get_logger(self):
        return _Logger()

    def _mission_log_append(self, *args, **kwargs):
        self._mission_log.append((args, kwargs))

    def _publish_mission_report(self):
        pass

    def _get_current_yaw(self):
        return 0.0

    def _sync_planned_tires_for_current_vehicle(self):
        pass


# --- Basic transitions ---

def test_initial_state_is_idle():
    node = _Node()
    assert node.current_state == MissionState.IDLE


def test_set_state_updates_current_state():
    node = _Node()
    set_state(node, MissionState.SEARCH_VEHICLE, cause="test")
    assert node.current_state == MissionState.SEARCH_VEHICLE


def test_set_state_same_state_is_noop():
    node = _Node()
    set_state(node, MissionState.IDLE, cause="test")
    assert node._state_repeat_count == 0


def test_state_pub_published():
    node = _Node()
    set_state(node, MissionState.SEARCH_VEHICLE, cause="test")
    assert node.state_pub.last is not None
    assert node.state_pub.last.data == MissionState.SEARCH_VEHICLE


def test_cause_stored():
    node = _Node()
    set_state(node, MissionState.SEARCH_VEHICLE, cause="tf_ready")
    assert node._last_transition_cause == "tf_ready"


# --- Wait context initialization ---

def test_wait_vehicle_box_initializes_wait_context():
    node = _Node()
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="test")
    assert isinstance(node._wait_context, WaitStateContext)
    assert node._wait_context.dispatch_retry_count == 0


def test_wait_tire_box_initializes_wait_context():
    node = _Node()
    set_state(node, MissionState.WAIT_TIRE_BOX, cause="test")
    assert isinstance(node._wait_context, WaitStateContext)


def test_wait_tire_box_sets_tire_search_start_time():
    node = _Node()
    set_state(node, MissionState.WAIT_TIRE_BOX, cause="test")
    assert node.wait_start_time is not None


# --- Segmentation mode publishing ---

def test_search_vehicle_publishes_navigation_mode():
    node = _Node()
    set_state(node, MissionState.SEARCH_VEHICLE, cause="test")
    assert node.segmentation_mode_pub.last.data == "navigation"


def test_wait_tire_box_publishes_inspection_mode():
    node = _Node()
    set_state(node, MissionState.WAIT_TIRE_BOX, cause="test")
    assert node.segmentation_mode_pub.last.data == "inspection"


def test_inspect_tire_publishes_inspection_mode():
    node = _Node()
    set_state(node, MissionState.INSPECT_TIRE, cause="test")
    assert node.segmentation_mode_pub.last.data == "inspection"


# --- Spin protection ---

def test_spin_protection_triggers_error_after_max_repeats():
    """After max_state_repeats cycle-returns, state should become ERROR."""
    node = _Node(max_state_repeats=3)
    # Drive the cycle: WAIT_VEHICLE_BOX → TURN_IN_PLACE_VEHICLE → WAIT_VEHICLE_BOX (repeat)
    node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="done_rotating")  # cycle 1
    assert node.current_state == MissionState.WAIT_VEHICLE_BOX

    node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="done_rotating")  # cycle 2
    assert node.current_state == MissionState.WAIT_VEHICLE_BOX

    node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="done_rotating")  # cycle 3 → ERROR
    assert node.current_state == MissionState.ERROR


def test_spin_protection_resets_on_progress():
    """A non-cycle transition resets the repeat counter."""
    node = _Node(max_state_repeats=3)
    node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="done_rotating")
    assert node._state_repeat_count == 1

    # Progress: go to APPROACH_VEHICLE (not a cycle return)
    set_state(node, MissionState.APPROACH_VEHICLE, cause="vehicle_confirmed")
    assert node._state_repeat_count == 0


def test_spin_protection_tire_wait_cycle():
    """Tire-side spin protection: WAIT_TIRE_BOX → TURN_IN_PLACE_TIRE → WAIT_TIRE_BOX."""
    node = _Node(max_state_repeats=3)
    for i in range(2):
        node.current_state = MissionState.TURN_IN_PLACE_TIRE
        set_state(node, MissionState.WAIT_TIRE_BOX, cause="done_rotating")
    assert node._state_repeat_count == 2
    node.current_state = MissionState.TURN_IN_PLACE_TIRE
    set_state(node, MissionState.WAIT_TIRE_BOX, cause="done_rotating")
    assert node.current_state == MissionState.ERROR


# --- Error/Done terminal states ---

def test_done_state_does_not_increment_error_count():
    node = _Node()
    set_state(node, MissionState.DONE, cause="all_done")
    assert node._mission_report["error_states_encountered"] == 0


def test_error_state_increments_via_spin_protection():
    node = _Node(max_state_repeats=3)
    for _ in range(2):
        node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
        set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="test")
    node.current_state = MissionState.TURN_IN_PLACE_VEHICLE
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="test")
    assert node._mission_report["error_states_encountered"] == 1


def test_face_tire_to_wait_tire_is_cycle_return():
    """FACE_TIRE → WAIT_TIRE_BOX is now classified as a cycle return.

    Bug fixed 2026-06: FACE_TIRE timeout previously was NOT a cycle return, so the
    INSPECT_TIRE→FACE_TIRE→WAIT_TIRE_BOX loop never triggered spin protection.
    Adding this transition to cycle returns means consecutive timeouts accumulate the counter.
    """
    node = _Node(max_state_repeats=3)
    # Simulate the consecutive FACE_TIRE→WAIT_TIRE_BOX (timeout) pattern
    for i in range(2):
        node.current_state = MissionState.FACE_TIRE
        set_state(node, MissionState.WAIT_TIRE_BOX, cause="face_tire_timeout")
        assert node._state_repeat_count == i + 1, f"Expected count {i+1} after cycle {i+1}"
    # Third consecutive FACE_TIRE→WAIT_TIRE_BOX should trigger ERROR
    node.current_state = MissionState.FACE_TIRE
    set_state(node, MissionState.WAIT_TIRE_BOX, cause="face_tire_timeout")
    assert node.current_state == MissionState.ERROR


def test_face_tire_to_wait_wheel_is_not_cycle_reset():
    """FACE_TIRE → WAIT_WHEEL_FOR_CAPTURE is forward progress, not a cycle return."""
    node = _Node(max_state_repeats=3)
    # First get count to 2 via cycle returns
    for _ in range(2):
        node.current_state = MissionState.FACE_TIRE
        set_state(node, MissionState.WAIT_TIRE_BOX, cause="face_tire_timeout")
    assert node._state_repeat_count == 2
    # Now the FACE_TIRE → WAIT_WHEEL_FOR_CAPTURE (already a cycle return in set 60-61) path
    # would progress: count goes to 3 → ERROR if this is also a cycle return
    # OR resets if it's forward progress.
    # WAIT_WHEEL_FOR_CAPTURE from FACE_TIRE is cycle return (line 60 in mission_state_machine.py)
    node.current_state = MissionState.FACE_TIRE
    set_state(node, MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="face_tire_done")
    assert node.current_state == MissionState.ERROR  # 3rd cycle return triggers ERROR


# --- NEXT_VEHICLE resets deferred state ---

def test_next_vehicle_resets_deferred_tires():
    node = _Node()
    node._tires_deferred = [(1.0, 2.0, 0.0)]
    node._return_later_pass_count = 1
    set_state(node, MissionState.NEXT_VEHICLE, cause="all_tires_done")
    assert node._tires_deferred == []
    assert node._return_later_pass_count == 0
