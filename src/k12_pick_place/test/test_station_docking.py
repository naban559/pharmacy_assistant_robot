import math
import threading
import time
from types import SimpleNamespace as N
import pytest
from geometry_msgs.msg import TransformStamped
from k12_pick_place.amms_task_manager import AmmsTaskManager as Manager


def fake_node(poses):
    sequence=iter(poses)
    commands=[]; alignments=[]
    node=N(_waypoint=lambda name:(0.,-2.25,-math.pi/2),_docking_pivot_x=lambda:.195,
           _physical_base_pose=lambda:next(sequence),
           _docking_clearance=lambda base:None,
           _stop_base=lambda:commands.append((0.,0.)),
           tracker_cmd_vel_pub=N(publish=lambda t:commands.append((t.linear.x,t.angular.z))),
           _verify_and_align_heading=lambda *a,**kw:alignments.append(a) or True,
           get_logger=lambda:N(info=lambda *a:None))
    return node, commands, alignments


def test_local_dock_uses_one_heading_alignment(monkeypatch):
    monkeypatch.setattr('k12_pick_place.amms_task_manager.time.sleep',lambda _:None)
    monkeypatch.setattr('k12_pick_place.amms_task_manager.rclpy.ok',lambda:True)
    node,commands,alignments=fake_node([(0.,-1.9,-math.pi/2),(0.,-2.245,-math.pi/2),(0.,-2.245,-math.pi/2)])
    Manager._dock_to_station(node)
    assert len(alignments)==1
    assert any(v==.05 and abs(w)<1e-9 for v,w in commands)
    assert commands[-1]==(0.,0.)


@pytest.mark.parametrize('pose,reason',[(None,'unavailable'),((0.,-2.28,-math.pi/2),'overshot'),((.6,-2.,0.),'bounded')])
def test_bad_pose_stops_without_motion(monkeypatch,pose,reason):
    monkeypatch.setattr('k12_pick_place.amms_task_manager.rclpy.ok',lambda:True)
    node,commands,_=fake_node([pose])
    with pytest.raises(RuntimeError,match=reason):Manager._dock_to_station(node)
    assert all(v==w==0. for v,w in commands)


def test_lidar_checks_full_robot_footprint():
    transform=TransformStamped();transform.transform.rotation.w=1.
    node=N(observer=N(get_pose=lambda *a,**k:{'position':(0.,-2.75,.05)}),
           _scan_lock=threading.Lock(),_scan_wall_time=time.monotonic(),
           _latest_scan=N(ranges=[.2],range_min=.05,range_max=10.,angle_min=math.pi,angle_increment=.1,header=N(frame_id='laser')),
           tf_buffer=N(lookup_transform=lambda *a,**k:transform))
    with pytest.raises(RuntimeError,match='Lidar obstacle'):Manager._docking_clearance(node,(0.,-2.2,-math.pi/2))
    node._latest_scan.ranges=[1.]
    Manager._docking_clearance(node,(0.,-2.2,-math.pi/2))
    with pytest.raises(RuntimeError,match='station clearance'):Manager._docking_clearance(node,(0.,-2.34,-math.pi/2))


@pytest.mark.parametrize('start',[(-.24,-1.684,-1.669),(.24,-1.8,-1.5),(0.,-1.9,-1.57),(-.1,-1.8,-2.)])
def test_docking_converges_with_measured_rotation_centre(start):
    from k12_pick_place.station_docking import docking_command
    x,y,yaw=start
    for i in range(2000):
        v,w,rho,heading=docking_command((x,y,yaw),(0.,-2.25,-math.pi/2),.195)
        assert 0. <= v <= .05 and abs(w) <= .22
        dt=.02
        # Observed skid-steer kinematics: front axle is the rotation centre.
        x += (v*math.cos(yaw)+.195*w*math.sin(yaw))*dt
        y += (v*math.sin(yaw)-.195*w*math.cos(yaw))*dt
        yaw += w*dt
        assert y-(.255*abs(math.sin(yaw))+.17*abs(math.cos(yaw))) > -2.575
        if rho<.008 and abs(heading)<.025:
            break
    else:
        pytest.fail('Docking failed to converge within 40 simulation seconds')
    assert math.hypot(x,y+2.25)<.015
    assert abs(yaw+math.pi/2)<.03
