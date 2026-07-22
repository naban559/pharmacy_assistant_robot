import pytest

from k12_pick_place.waypoint_manager import coerce_waypoints


def test_coerce_waypoints_uses_configured_values():
    waypoints = coerce_waypoints(
        ['home', 'pick_station'],
        {'pick_station': [2.5, -1.2, 0.0]},
        {'home': [0.0, 0.0, 0.0]},
    )

    assert waypoints['home'] == (0.0, 0.0, 0.0)
    assert waypoints['pick_station'] == (2.5, -1.2, 0.0)


def test_coerce_waypoints_rejects_bad_shape():
    with pytest.raises(ValueError):
        coerce_waypoints(['bad'], {'bad': [1.0, 2.0]}, {})
