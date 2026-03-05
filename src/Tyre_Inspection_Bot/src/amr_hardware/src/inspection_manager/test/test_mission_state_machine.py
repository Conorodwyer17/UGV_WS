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


class _Node:
    def __init__(self):
        self.current_state = MissionState.IDLE
        self._last_transition_cause = "init"
        self._state_repeat_count = 0
        self._mission_report = {"error_states_encountered": 0}
        self.state_pub = _Pub()
        self.segmentation_mode_pub = _Pub()
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
        self._mission_log = []

    def get_parameter(self, name):
        vals = {"max_state_repeats": 3}
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


def test_wait_state_initializes_wait_context():
    node = _Node()
    set_state(node, MissionState.WAIT_VEHICLE_BOX, cause="test")
    assert isinstance(node._wait_context, WaitStateContext)
    assert node._wait_context.dispatch_retry_count == 0
