"""Navigation must not begin while the arm is busy or failed to fold."""
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from k12_pick_place import amms_task_manager as manager


@pytest.fixture(autouse=True)
def source_package_share(monkeypatch):
    from pathlib import Path
    monkeypatch.setattr(manager, 'get_package_share_directory',
                        lambda name: str(Path(__file__).parents[1]))


def test_busy_startup_arm_is_retried_before_driving(monkeypatch):
    monkeypatch.setattr(manager.rclpy, 'ok', lambda: True)
    sleeps = []
    monkeypatch.setattr(manager.time, 'sleep', sleeps.append)
    calls = []
    def fold(client, label):
        calls.append(label)
        if len(calls) == 1:
            raise RuntimeError('Service fold arm failed: Manipulator is already executing.')
    node = SimpleNamespace(fold_client=object(), _call_trigger=fold)
    manager.AmmsTaskManager._fold_before_driving(node)
    assert calls == ['fold arm', 'fold arm']
    assert sleeps == [.25]


def test_failed_fold_aborts_instead_of_driving(monkeypatch):
    monkeypatch.setattr(manager.rclpy, 'ok', lambda: True)
    def fold(client, label):
        raise RuntimeError('MoveIt failed safe carry')
    node = SimpleNamespace(fold_client=object(), _call_trigger=fold)
    with pytest.raises(RuntimeError, match='MoveIt failed'):
        manager.AmmsTaskManager._fold_before_driving(node)


def test_navigation_cancellation_is_acknowledged_before_grasp():
    stopped = []
    terminal = SimpleNamespace(result=lambda:SimpleNamespace(status=5))
    cancellation = object()
    waited = []
    node = SimpleNamespace(_stop_base=lambda:stopped.append(True),
                           _wait_for_future=lambda future,timeout:(waited.append(future) or True))
    handle = SimpleNamespace(cancel_goal_async=lambda:cancellation)
    manager.AmmsTaskManager._stop_navigation_goal(node,handle,terminal)
    assert waited == [cancellation,terminal]
    assert len(stopped) == 2


def test_unacknowledged_navigation_cancel_blocks_followup_motion():
    node = SimpleNamespace(_stop_base=lambda:None, _wait_for_future=lambda *args:False)
    handle = SimpleNamespace(cancel_goal_async=lambda:object())
    with pytest.raises(RuntimeError,match='not acknowledged'):
        manager.AmmsTaskManager._stop_navigation_goal(node,handle,object())
    assert node.navigation_fault


@pytest.mark.parametrize('label,tree', [
    ('reachable grasp approach', 'approach_grasp.xml'),
    ('pickup staging', 'navigate.xml'),
])
def test_fine_approach_stops_on_reach_before_waypoint(monkeypatch, label, tree):
    from geometry_msgs.msg import PoseStamped
    monkeypatch.setattr(manager.rclpy, 'ok', lambda:True)
    pending = SimpleNamespace(done=lambda:False)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda:pending)
    sent = SimpleNamespace(result=lambda:handle)
    cancelled = []
    selected = []
    goals = []
    def detection(reachable, object_id):
        selected.append((reachable, object_id))
        return {'reachable': True}
    node = SimpleNamespace(
        nav_client=SimpleNamespace(wait_for_server=lambda **kwargs:True,
                                   send_goal_async=lambda goal:(goals.append(goal) or sent)),
        _wait_for_future=lambda *args:True,
        _select_valid_detection=detection,
        _stop_navigation_goal=lambda *args:cancelled.append(args),
        get_logger=lambda:SimpleNamespace(info=lambda *args:None))
    manager.AmmsTaskManager._navigate_to_pose_once(
        node, PoseStamped(), label, 10, 'red_cube')
    assert goals[0].behavior_tree.endswith('/config/' + tree)
    assert selected == [(True, 'red_cube')]
    assert cancelled == [(handle,pending)]
    assert not node.base_motion_active


@pytest.mark.parametrize('distance,should_stop', [(.20,True),(.45,False)])
def test_visible_cube_only_finishes_staging_near_the_waypoint(monkeypatch,distance,should_stop):
    from geometry_msgs.msg import PoseStamped
    monkeypatch.setattr(manager.rclpy,'ok',lambda:True)
    monkeypatch.setattr(manager.time,'sleep',lambda seconds:None)
    done = iter([False,True])
    result = SimpleNamespace(done=lambda:next(done), result=lambda:SimpleNamespace(status=4))
    handle = SimpleNamespace(accepted=True,get_result_async=lambda:result)
    selected=[]
    cancelled=[]
    def detection(reachable,object_id):
        selected.append((reachable,object_id))
        return {'reachable':False}
    node=SimpleNamespace(
        nav_client=SimpleNamespace(wait_for_server=lambda **kw:True,
                                   send_goal_async=lambda goal:SimpleNamespace(result=lambda:handle)),
        _wait_for_future=lambda *args:True,
        _select_valid_detection=detection,
        _current_base_pose=lambda:(distance,0,0),
        _stop_navigation_goal=lambda *args:cancelled.append(True),
        get_logger=lambda:SimpleNamespace(info=lambda *args:None))
    manager.AmmsTaskManager._navigate_to_pose_once(
        node,PoseStamped(),'pickup staging',10,'red_cube',True)
    assert selected == [(False,'red_cube')]
    assert bool(cancelled) == should_stop


def test_pickup_goal_tolerance_fits_complete_grasp_envelope():
    """Nav2 may finish short of its goal; that point must still admit a pick."""
    import math
    from pathlib import Path
    import yaml
    from k12_pick_place.ik import PlanarArmIK
    config_dir = Path(__file__).parents[1] / 'config'
    nav = yaml.safe_load((config_dir / 'nav2_amms.yaml').read_text())['controller_server']['ros__parameters']
    task = yaml.safe_load((config_dir / 'amms_waypoints.yaml').read_text())['amms_task_manager']['ros__parameters']
    tolerance = nav['grasp_goal_checker']['xy_goal_tolerance']
    yaw = nav['grasp_goal_checker']['yaw_goal_tolerance']
    assert nav['FineApproach']['xy_goal_tolerance'] == tolerance
    # World cube centre, base footprint height and arm mount from the URDF.
    z = .1625 - .057 - .142 + task['object_grasp_z_bias']
    distance = task['object_nav_standoff_distance']
    ik = PlanarArmIK()
    assert ik.pick_plan((distance + .20 - .12, 0, z)) is None
    for angle in (-yaw, 0, yaw):
        for error in (-tolerance, tolerance):
            d = distance + error
            grasp = (d * math.cos(angle) - .12, d * math.sin(angle), z)
            assert ik.pick_plan(grasp) is not None


def test_approach_requires_a_detection_received_after_stopping(monkeypatch):
    from builtin_interfaces.msg import Time
    monkeypatch.setattr(manager.time, 'monotonic', lambda: 100.)
    params = {'max_reposition_attempts': 1, 'use_navigation': True,
              'object_nav_standoff_distance': .39, 'frame_id': 'map'}
    seed = {'object_id': 'red_cube', 'object': {'id': 'red_cube'},
            'map_position': (-2.88, 2.62, .1625), 'reachable': False}
    fresh = dict(seed, reachable=True)
    calls = []
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        _update_candidate_for_current_pose=lambda c: c,
        _candidate_is_valid=lambda c, required: c['reachable'],
        _current_base_pose=lambda: (-2.88, 1.95, 1.57),
        _navigate_to_pose=lambda *a, **kw: None, _stop_base=lambda: None,
        _object_id=manager.AmmsTaskManager._object_id,
        _wait_for_valid_detection=lambda *a, **kw: (calls.append(kw) or fresh))
    assert manager.AmmsTaskManager._acquire_reachable_object(node, seed) is fresh
    assert calls[0]['min_detection_time'] == 100.
    # No cached pose may substitute for a lost post-motion observation.
    node._wait_for_valid_detection = lambda *a, **kw: None
    assert manager.AmmsTaskManager._acquire_reachable_object(node, seed) is None


def test_confirmation_does_not_fall_back_to_stale_target(monkeypatch):
    monkeypatch.setattr(manager.time, 'monotonic', lambda: 100.)
    requested = []
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=2.),
        _wait_for_valid_detection=lambda *a, **kw: (requested.append(kw) or None))
    assert manager.AmmsTaskManager._confirm_fresh_target(
        node, 'red_cube', {'reachable': True}) is None
    assert requested[0]['min_detection_time'] == 100.


def test_navigation_trees_select_matching_controllers_and_checkers():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    import yaml
    config = Path(__file__).parents[1] / 'config'
    nav = yaml.safe_load((config / 'nav2_amms.yaml').read_text())['controller_server']['ros__parameters']
    for filename, controller, checker in [
        ('navigate.xml', 'FollowPath', 'general_goal_checker'),
        ('approach_grasp.xml', 'FineApproach', 'grasp_goal_checker'),
    ]:
        follow = ET.parse(config / filename).find('.//FollowPath')
        assert follow.attrib['controller_id'] == controller
        assert follow.attrib['goal_checker_id'] == checker
        assert controller in nav['controller_plugins']
        assert checker in nav['goal_checker_plugins']
    assert nav['FineApproach']['max_vel_x'] < nav['FollowPath']['max_vel_x']
