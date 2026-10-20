import numpy as np
import pytest

pytest.importorskip("rclpy")
from k12_pick_place.object_detector import known_cube_center


@pytest.mark.parametrize("yaw", [0., .7, -1.2])
def test_visible_face_center_rotates_back_to_camera(yaw):
    side = .025
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    center = np.array([.3,.2,.5])
    a,b = np.meshgrid(np.linspace(-side/2,side/2,30), np.linspace(-side/2,side/2,30))
    points = center + np.column_stack((a.ravel(),b.ravel(),np.full(a.size,-side/2)))
    result = known_cube_center(points @ rotation, rotation, side)
    assert np.allclose(result, rotation.T @ center, atol=.0005)


def test_rejects_insufficient_or_oversized_surface():
    assert known_cube_center(np.zeros((5,3)),np.eye(3),.025) is None
    points = np.column_stack((np.linspace(0,.1,100),np.zeros(100),np.ones(100)))
    assert known_cube_center(points,np.eye(3),.025) is None
