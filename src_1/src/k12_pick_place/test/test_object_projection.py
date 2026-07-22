from k12_pick_place.object_detector import detection_confidence, project_pixel_to_ros_optical


def test_project_pixel_to_ros_optical_center_pixel():
    point = project_pixel_to_ros_optical(320, 240, 2.0, 600.0, 600.0, 320.0, 240.0)

    assert point == (0.0, 0.0, 2.0)


def test_project_pixel_to_ros_optical_offset_pixel():
    point = project_pixel_to_ros_optical(620, 90, 2.0, 600.0, 600.0, 320.0, 240.0)

    assert point == (1.0, -0.5, 2.0)


def test_project_pixel_rejects_invalid_intrinsics():
    assert project_pixel_to_ros_optical(320, 240, 2.0, 0.0, 600.0, 320.0, 240.0) is None


def test_detection_confidence_penalizes_small_or_distant_contours():
    strong = detection_confidence(area=650.0, depth=1.0, confidence_area_px=500.0, max_depth_m=4.5)
    weak = detection_confidence(area=80.0, depth=4.0, confidence_area_px=500.0, max_depth_m=4.5)

    assert 0.0 <= weak < strong <= 1.0
