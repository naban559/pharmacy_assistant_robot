"""Completion requires measured arrival and successful, stopped base actions."""
from concurrent.futures import Future
from types import MethodType, SimpleNamespace
import threading

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from k12_pick_place import amms_task_manager as manager


def quiet_logger():
    return SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                           error=lambda *a: None)


def test_failed_return_propagates_without_home_fallback():
    calls = []
    failure = RuntimeError('return navigation aborted')

    def navigate(pose, label):
        calls.append(label)
        raise failure

    node = SimpleNamespace(
        _initial_pose=(1., 2., .3),
        _fold_before_driving=lambda: calls.append('fold'),
        _pose_stamped=lambda pose: pose,
        _navigate_to_pose=navigate,
        _navigate_to_waypoint=lambda name: calls.append(name),
    )
    with pytest.raises(RuntimeError) as caught:
        manager.AmmsTaskManager._return_to_initial(node)
    assert caught.value is failure
    assert calls == ['fold', 'return_to_initial']


def test_missing_initial_pose_prevents_starting_pickup():
    states = []
    motions = []
    node = SimpleNamespace(
        _ABC_LABELS={'A': 'Red pickup'}, _ABC_WAYPOINTS={'A': 'pickup_a'},
        get_logger=quiet_logger, _set_state=states.append,
        _current_base_pose=lambda: None,
        _fold_before_driving=lambda: motions.append('fold'),
        _navigate_to_waypoint=lambda *a, **kw: motions.append('navigate'),
    )
    with pytest.raises(RuntimeError, match='Could not capture initial pose'):
        manager.AmmsTaskManager._run_pick_place_mission(node, 'A')
    assert states == [manager.State.CAPTURE_INITIAL_POSE]
    assert motions == []


def test_missing_initial_pose_cannot_report_return_success():
    node = SimpleNamespace(_initial_pose=None)
    with pytest.raises(RuntimeError, match='Initial pose was not captured'):
        manager.AmmsTaskManager._return_to_initial(node)


@pytest.mark.parametrize('label,base', [
    ('place_station', (.03, 0., 0.)),
    ('place_station', (0., 0., .06)),
    ('return_to_initial', (.09, 0., 0.)),
    ('return_to_initial', (0., 0., .16)),
    ('return_to_initial', None),
])
def test_action_success_requires_measured_arrival(monkeypatch, label, base):
    """A succeeded action outside pose tolerance must retry and ultimately fail."""
    monkeypatch.setattr(manager.time, 'sleep', lambda *a: None)
    params = {
        'nav_retry_count': 2, 'nav_retry_wait_sec': 0., 'nav_timeout_sec': 10.,
        'post_navigation_settle_sec': 0.,
        'docking_arrival_xy_tolerance_m': .02,
        'docking_arrival_yaw_tolerance_rad': .05,
        'nav_arrival_xy_tolerance_m': .08,
        'nav_arrival_yaw_tolerance_rad': .15,
    }
    attempts = []
    clears = []
    node = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=quiet_logger,
        _navigate_to_pose_once=lambda *a: attempts.append(a),
        _current_base_pose=lambda: base, _stop_base=lambda: None,
        _clear_nav_costmaps=clears.append,
    )
    node._verify_arrival = MethodType(manager.AmmsTaskManager._verify_arrival, node)
    pose = PoseStamped()
    pose.pose.orientation.w = 1.
    with pytest.raises(RuntimeError, match='failed after 2 attempts'):
        manager.AmmsTaskManager._navigate_to_pose(node, pose, label)
    assert len(attempts) == 2
    assert clears == [label]


@pytest.mark.parametrize('busy,pending,state,fault', [
    (True, None, manager.State.WAIT_FOR_COMMAND, False),
    (False, 'A', manager.State.WAIT_FOR_COMMAND, False),
    (False, None, manager.State.SYSTEM_STARTUP, False),
    (False, None, manager.State.PICK_OBJECT, False),
    (False, None, manager.State.WAIT_FOR_COMMAND, True),
])
def test_command_cannot_overwrite_pending_or_unready_mission(busy, pending, state, fault):
    node = SimpleNamespace(
        _mission_busy=busy, _pending_command=pending, _state=state,
        navigation_fault=fault, _command_event=threading.Event(), get_logger=quiet_logger,
    )
    response = SimpleNamespace(success=None, message='')
    manager.AmmsTaskManager._handle_goto(node, 'B', None, response)
    assert response.success is False
    assert node._pending_command == pending
    assert not node._command_event.is_set()


def test_ready_command_is_accepted_and_wakes_mission_worker():
    node = SimpleNamespace(
        _mission_busy=False, _pending_command=None,
        _state=manager.State.WAIT_FOR_COMMAND,
        navigation_fault=False, _command_event=threading.Event(), get_logger=quiet_logger,
    )
    response = SimpleNamespace(success=None, message='')
    manager.AmmsTaskManager._handle_goto(node, 'B', None, response)
    assert response.success is True
    assert node._pending_command == 'B'
    assert node._command_event.is_set()


def backup_node(status=4, result_ready=True):
    params = {'use_navigation': True, 'distance': .2, 'speed': .1,
              'timeout': 10., 'nav_timeout_sec': 90.}
    events = []
    result = SimpleNamespace(result=lambda: SimpleNamespace(status=status))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
    sent = SimpleNamespace(result=lambda: handle)

    def wait(future, timeout):
        events.append(('wait', future, timeout))
        return True if future is sent else result_ready

    node = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        backup_client=SimpleNamespace(wait_for_server=lambda **kw: True,
                                      send_goal_async=lambda goal: sent),
        _wait_for_future=wait,
        _stop_navigation_goal=lambda goal, future: events.append(('cancel', goal, future)),
        _stop_base=lambda: events.append(('stop',)),
    )
    return node, events, handle, result


@pytest.mark.parametrize('status', [5, 6])
def test_failed_backup_aborts_and_stops_base(status):
    node, events, _, _ = backup_node(status=status)
    with pytest.raises(RuntimeError, match=f'backup failed: status={status}'):
        manager.AmmsTaskManager._backup(node, 'distance', 'speed', 'timeout', 'backup')
    assert node.base_motion_active is False
    assert events[-1] == ('stop',)


def test_backup_timeout_cancels_and_waits_before_subsequent_motion():
    node, events, handle, result = backup_node(result_ready=False)
    with pytest.raises(RuntimeError, match='backup timed out'):
        manager.AmmsTaskManager._backup(node, 'distance', 'speed', 'timeout', 'backup')
    assert ('wait', result, 90.) in events
    assert events[-2:] == [('cancel', handle, result), ('stop',)]
    assert node.base_motion_active is False


def test_unacknowledged_backup_cancellation_keeps_navigation_fault():
    node, events, handle, result = backup_node(result_ready=False)
    node._stop_navigation_goal = MethodType(manager.AmmsTaskManager._stop_navigation_goal, node)
    handle.cancel_goal_async = lambda: object()
    with pytest.raises(RuntimeError, match='cancellation was not acknowledged'):
        manager.AmmsTaskManager._backup(node, 'distance', 'speed', 'timeout', 'backup')
    assert node.navigation_fault is True
    assert node.base_motion_active is False
    assert events[-1] == ('stop',)


def test_late_backup_acceptance_is_cancelled_after_send_timeout():
    node, _, _, _ = backup_node()
    future = Future()
    cancelled = []
    node.backup_client.send_goal_async = lambda goal: future
    node._wait_for_future = lambda *a: False
    with pytest.raises(RuntimeError, match='Timed out sending backup'):
        manager.AmmsTaskManager._backup(node, 'distance', 'speed', 'timeout', 'backup')
    assert node.navigation_fault is True
    future.set_result(SimpleNamespace(accepted=True,
                                     cancel_goal_async=lambda: cancelled.append(True)))
    assert cancelled == [True]
