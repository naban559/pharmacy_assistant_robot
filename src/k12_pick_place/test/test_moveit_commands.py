"""Exercise the actual request construction with fake action/service transport."""
from types import SimpleNamespace as NS
import math
import time
import pytest

pytest.importorskip('rclpy')
from k12_pick_place.moveit_pick_place_commander import MoveItPickPlaceCommander as Commander
from k12_pick_place.pick_place_commander import PickPlaceCommander
from k12_pick_place.ik import PlanarArmIK

NAMES = ['arm_base_yaw_joint', 'arm_shoulder_pitch_joint', 'arm_elbow_pitch_joint',
         'arm_wrist_pitch_joint', 'arm_wrist_roll_joint']


class Future:
    def __init__(self, result): self.value = result
    def result(self): return self.value
    def done(self): return True


def parameters(name):
    return NS(value={'move_group_name': 'arm', 'move_group_planning_time': 10.0,
                     'move_group_num_planning_attempts': 3,
                     'move_group_velocity_scale': .2, 'move_group_acceleration_scale': .2,
                     'cartesian_position_tolerance': .003,
                     'cartesian_orientation_tolerance': .03,
                     'joint_feedback_timeout_sec': 1.0}[name])


def test_both_executables_share_the_checked_engine():
    assert PickPlaceCommander is Commander


def test_moveit_receives_five_joint_constraints_no_position_only_goal():
    q = (0.1, .8, -.2, .7, .6)
    sent = []
    handle = NS(accepted=True, get_result_async=lambda: Future(NS(status=4, result=NS(error_code=NS(val=1)))))
    client = NS(wait_for_server=lambda **kw: True,
                send_goal_async=lambda goal: (sent.append(goal) or Future(handle)))
    node = NS(motion_fault=False, ik=PlanarArmIK(), joint_names=NAMES, _arm_feedback=lambda:q,
              _move_group_action=client, get_parameter=parameters, _wait_for_future=lambda *args: True)
    Commander._move_joints(node, q, 3, 'pre-grasp')
    request = sent[0].request
    constraints = request.goal_constraints[0]
    assert [c.position for c in constraints.joint_constraints] == list(q)
    assert [c.joint_name for c in constraints.joint_constraints] == NAMES
    assert not constraints.position_constraints
    assert not constraints.orientation_constraints


def test_unavailable_moveit_does_not_send_direct_motion():
    node = NS(motion_fault=False, ik=PlanarArmIK(), _arm_feedback=lambda:(0,)*5,
              _move_group_action=NS(wait_for_server=lambda **kw:False))
    with pytest.raises(RuntimeError, match='no direct'):
        Commander._move_joints(node, (0,)*5, 3, 'grasp')


def test_action_success_waits_for_joint_feedback_to_settle():
    q = (0.1, .8, -.2, .7, .6)
    lagging = (q[0]+.03,) + q[1:]
    feedback = iter([q, lagging, q])
    sleeps = []
    handle = NS(accepted=True, get_result_async=lambda: Future(
        NS(status=4, result=NS(error_code=NS(val=1)))))
    node = NS(motion_fault=False, ik=PlanarArmIK(), joint_names=NAMES,
              _arm_feedback=lambda:next(feedback), get_parameter=parameters,
              _wait_for_future=lambda *args:True, _sleep=sleeps.append,
              _move_group_action=NS(wait_for_server=lambda **kw:True,
                                    send_goal_async=lambda goal:Future(handle)))
    Commander._move_joints(node, q, 3, 'pre-grasp')
    assert sleeps == [.02]


def test_missing_or_stale_feedback_is_rejected():
    node = NS(joint_names=NAMES, joint_feedback_times={}, get_parameter=parameters)
    with pytest.raises(RuntimeError, match='stale'):
        Commander._arm_feedback(node)


def test_descent_copies_measured_roll_and_uses_bounded_quintic_timing():
    ik = PlanarArmIK()
    q = ik.solve((.30,0,-.0265), pitch=math.pi/2, roll=.62)
    final = ik.vertical_path(q, -.0765)[-1]
    state = [q]
    sent = []
    def execute(trajectory, duration, label, cartesian_start):
        sent.append(trajectory)
        assert cartesian_start == q
        state[0] = final
    node = NS(motion_fault=False, ik=ik, joint_names=NAMES, _arm_feedback=lambda:state[0],
              get_parameter=parameters, _validate_joint_path=lambda *args, **kw:None,
              get_logger=lambda:NS(info=lambda *args:None),
              _send_joint_trajectory_goal=execute, _verify_tcp_position=lambda *args:None)
    Commander._vertical_move(node, (.30,0,-.0765), 'grasp')
    points = sent[0].points
    for p in points:
        assert p.positions[4] == pytest.approx(.62)
        assert list(p.velocities) == [0.0]*5
        assert list(p.accelerations) == [0.0]*5
    for a,b in zip(points,points[1:]):
        ta = a.time_from_start.sec + a.time_from_start.nanosec*1e-9
        tb = b.time_from_start.sec + b.time_from_start.nanosec*1e-9
        delta = max(abs(x-y) for x,y in zip(a.positions,b.positions))
        assert 1.875*delta/(tb-ta) <= .4+1e-7
        assert 5.774*delta/(tb-ta)**2 <= .8+1e-7


@pytest.mark.parametrize('body,depth,allow,expected', [
    ('amms_support_0', .0001, True, True),
    ('amms_support_0', .003, True, False),
    ('amms_support_0', .0001, False, False),
    ('forearm_link', .0001, True, False),
    ('amms_support_4', .0001, True, False),
])
def test_only_shallow_pickup_support_contact_is_allowed_at_liftoff(body,depth,allow,expected):
    ik = PlanarArmIK()
    q = ik.solve((.30,0,-.0765), pitch=math.pi/2)
    response = NS(valid=False, contacts=[NS(contact_body_1='amms_held_object',
                                          contact_body_2=body,depth=depth)])
    node = NS(ik=ik, joint_names=NAMES, latest_joint_positions=dict(zip(NAMES,q)),
              get_parameter=parameters, _wait_for_future=lambda *args:True,
              state_validity_client=NS(wait_for_service=lambda **kw:True,
                                       call_async=lambda request:Future(response)))
    if expected:
        Commander._validate_joint_path(node, [q], departing_support=allow)
    else:
        with pytest.raises(RuntimeError, match='collision'):
            Commander._validate_joint_path(node, [q], departing_support=allow)


def test_detection_timestamp_updates_on_detection_not_place_pose():
    node = NS(latest_time=0.0)
    Commander._on_detections(node, NS(data='{"objects": []}'))
    assert time.monotonic()-node.latest_time < .1


def test_gripper_wait_does_not_finish_at_old_three_mm_tolerance(monkeypatch):
    import k12_pick_place.moveit_pick_place_commander as module
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    positions={'left_gripper_joint':.0095,'right_gripper_joint':.0095}
    sleeps=[]
    def settle(seconds):
        sleeps.append(seconds)
        positions.update(left_gripper_joint=.0113,right_gripper_joint=.0113)
    node=NS(latest_joint_positions=positions,_publish_gripper=lambda *args,**kw:None,
            _sleep=settle)
    Commander._command_gripper_until_reached(node,.0115,.0115,'open')
    assert sleeps == [.10]


def test_slow_simulation_uses_controller_clock(monkeypatch):
    from trajectory_msgs.msg import JointTrajectory
    from k12_pick_place import moveit_pick_place_commander as module
    ticks = [0]
    result = NS(done=lambda:ticks[0] >= 20,
                result=lambda:NS(status=4,result=NS(error_code=0)))
    handle = NS(accepted=True,get_result_async=lambda:result)
    node = NS(arm_action_client=NS(wait_for_server=lambda **kw:True,
              send_goal_async=lambda goal:Future(handle)),
              _wait_for_future=lambda *args:True,
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=ticks[0]*100000000)),
              _arm_feedback=lambda:(0,)*5,
              _sleep=lambda delay:ticks.__setitem__(0,ticks[0]+1))
    monkeypatch.setattr(module.time,'monotonic',lambda:ticks[0]*1.0)
    Commander._send_joint_trajectory_goal(node,JointTrajectory(),2.,'slow simulation')
    assert ticks[0] == 20  # 20 wall seconds, only 2 controller seconds


def test_stalled_controller_clock_cancels_and_latches_fault(monkeypatch):
    from trajectory_msgs.msg import JointTrajectory
    from k12_pick_place import moveit_pick_place_commander as module
    ticks, cancelled = [0], []
    handle = NS(accepted=True,get_result_async=lambda:NS(done=lambda:False),
                cancel_goal_async=lambda:(cancelled.append(True) or Future(None)))
    node = NS(motion_fault=False,
              arm_action_client=NS(wait_for_server=lambda **kw:True,
              send_goal_async=lambda goal:Future(handle)),
              _wait_for_future=lambda *args:True,
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=0)),
              _arm_feedback=lambda:(0,)*5,
              _sleep=lambda delay:ticks.__setitem__(0,ticks[0]+1))
    monkeypatch.setattr(module.time,'monotonic',lambda:float(ticks[0]))
    with pytest.raises(RuntimeError,match='clock stalled'):
        Commander._send_joint_trajectory_goal(node,JointTrajectory(),2.,'paused simulation')
    assert cancelled == [True]
    assert node.motion_fault


def test_cartesian_goal_requires_measured_joint_convergence():
    from trajectory_msgs.msg import JointTrajectory
    sent = []
    handle = NS(accepted=True, get_result_async=lambda: Future(
        NS(status=4, result=NS(error_code=0))))
    node = NS(arm_action_client=NS(wait_for_server=lambda **kw: True,
              send_goal_async=lambda goal: (sent.append(goal) or Future(handle))),
              _wait_for_future=lambda *args: True,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)))
    Commander._send_joint_trajectory_goal(
        node, JointTrajectory(joint_names=NAMES), 2., 'grasp')
    goal = sent[0]
    assert [t.name for t in goal.goal_tolerance] == NAMES
    assert all(t.position == .002 and t.velocity == .01 for t in goal.goal_tolerance)
    assert goal.goal_time_tolerance.sec == 8


def test_blocked_descent_reports_controller_reason():
    from trajectory_msgs.msg import JointTrajectory
    handle = NS(accepted=True, get_result_async=lambda: Future(
        NS(status=6, result=NS(error_code=-5, error_string='goal position tolerance violated'))))
    node = NS(arm_action_client=NS(wait_for_server=lambda **kw: True,
              send_goal_async=lambda goal: Future(handle)),
              _wait_for_future=lambda *args: True,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)))
    with pytest.raises(RuntimeError, match='goal position tolerance violated'):
        Commander._send_joint_trajectory_goal(
            node, JointTrajectory(joint_names=NAMES), 2., 'grasp')


@pytest.mark.parametrize('bias', [.010, .012])
def test_place_tcp_preserves_cube_clearance_and_grasp_offset(bias):
    import threading
    checked = []
    # A 25 mm cube centered here has 3 mm clearance above the station top.
    center = (.23, 0., .30 + .0125 + .004)
    node = NS(hold_lock=threading.Lock(), held_entity_name='red_cube',
              observer=NS(get_pose=lambda *a, **k: {'position':(.23,0.,.2925), 'orientation':(0.,0.,0.,1.)},
                          world_point_in_frame=lambda node, point, frame: tuple(point)),
              get_logger=lambda: NS(info=lambda *a: None),
              get_parameter=lambda name: NS(value={
                  'place_table_frame': 'map', 'target_frame': 'arm_base_link',
                  'grasp_z_bias': bias}[name]),
              ik=NS(pick_plan=lambda point, **kw: (checked.append(point) or {})))
    tcp = Commander._current_place_pose(node)
    assert tcp[2] - bias - .0125 == pytest.approx(.304)
    assert tcp[:2] == center[:2]
    assert checked == [tcp]


@pytest.mark.parametrize('bias', [.010, .012])
def test_attached_collision_uses_actual_grasp_offset(bias):
    requests = []
    node = NS(get_parameter=lambda name: NS(value=bias), _apply_scene=requests.append)
    Commander._set_payload_collision(node, True)
    attached = requests[0].scene.robot_state.attached_collision_objects[0]
    assert attached.object.primitive_poses[0].position.x == pytest.approx(bias)


def test_transform_vector_from_map():
    # If arm base is facing -Y (yaw = -pi/2), vector (-0.08, -0.01, 0) in map rotates to arm base
    node = NS(
        get_parameter=lambda name: NS(value='map' if name == 'place_table_frame' else 'arm_base_link'),
        _transform_fixed_point=lambda frame, pt, target: (
            (pt[1], -pt[0], pt[2]) if frame == 'map' else pt
        )
    )
    vec = Commander._transform_vector_from_map(node, (-0.08, -0.01, 0.0), 'arm_base_link')
    assert vec is not None
    assert vec[0] == pytest.approx(-0.01)
    assert vec[1] == pytest.approx(0.08)



def test_retreat_scene_uses_settled_cube_pose_without_relaxing_collisions():
    import numpy as np
    requests=[]
    node=NS(inspect_scene_client=NS(wait_for_service=lambda **kw:False),
            observer=NS(get_pose=lambda *a,**kw:{'position':(-.08,-2.64,.1275),'orientation':(0.,0.,0.,1.)},
                        world_to_frame=lambda *a:np.eye(4)),
            get_parameter=lambda name:NS(value='arm_base_link'),
            _apply_scene=requests.append,_finger_feedback=lambda:(.0115,.0115),
            get_logger=lambda:NS(info=lambda *a:None))
    Commander._sync_released_cube(node,'cube_red')
    scene=requests[0].scene
    obj=scene.world.collision_objects[0]
    assert obj.id=='amms_held_object'
    assert obj.primitive_poses[0].position.z==pytest.approx(.1275)
    assert list(obj.primitives[0].dimensions)==[.025]*3
    assert not scene.allowed_collision_matrix.entry_names


@pytest.mark.parametrize('case,passes', [
    ('existing_touch', True), ('deep', False), ('increasing', False),
    ('never_clears', False), ('new_contact', False), ('other_body', False),
    ('not_verified', False), ('closed', False), ('still_held', False),
    ('lateral', False), ('rotating', False),
])
def test_supported_cube_retreat_only_departs_existing_microscopic_touch(case, passes):
    ik = PlanarArmIK()
    start = ik.solve((.30, 0., -.0565), pitch=math.pi/2)
    path = ik.vertical_path(start, -.0315)
    if case == 'lateral':
        path[-1] = ik.solve((.301, 0., -.0315), pitch=math.pi/2)
    if case == 'rotating':
        path[-1] = tuple(path[-1][:4]) + (.01,)
    def check(request):
        q = list(request.robot_state.joint_state.position)[:5]
        dz = ik.forward(q)[2] - ik.forward(start)[2]
        touching = dz < .01 or case == 'never_clears'
        if case == 'new_contact':
            touching = .003 < dz < .01
        depth = 2e-6 if case == 'deep' else 5e-7
        if case == 'increasing' and dz > .002:
            depth = 8e-7
        body = 'forearm_link' if case == 'other_body' else 'right_gripper'
        contact = NS(contact_body_1='amms_held_object', contact_body_2=body, depth=depth)
        return Future(NS(valid=not touching, contacts=[contact] if touching else []))
    node = NS(ik=ik, joint_names=NAMES, latest_joint_positions=dict(zip(NAMES,start)),
              get_parameter=parameters, _wait_for_future=lambda *a:True,
              _holding_object=lambda:case == 'still_held',
              _finger_feedback=lambda:(.010 if case == 'closed' else .0115, .0115),
              get_logger=lambda:NS(info=lambda *a:None),
              state_validity_client=NS(wait_for_service=lambda **kw:True, call_async=check))
    if passes:
        Commander._validate_joint_path(node, path, label='place retreat', released_support_verified=True)
    else:
        with pytest.raises(RuntimeError):
            Commander._validate_joint_path(node, path, label='place retreat',
                                           released_support_verified=case != 'not_verified')
