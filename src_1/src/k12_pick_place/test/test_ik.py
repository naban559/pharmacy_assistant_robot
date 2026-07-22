from k12_pick_place.ik import PlanarArmIK


def test_planar_ik_solves_reachable_pick_pose():
    ik = PlanarArmIK()
    solution = ik.solve((0.72, 0.0, 0.12))

    assert solution is not None
    assert len(solution) == 5


def test_planar_ik_rejects_unreachable_pose():
    ik = PlanarArmIK()

    assert ik.solve((2.0, 0.0, 0.12)) is None


def test_planar_ik_solves_adjusted_place_slots():
    ik = PlanarArmIK()

    for x, y in ((1.00, 0.12), (1.00, -0.12), (1.03, 0.06)):
        assert ik.solve((x, y, 0.12)) is not None
        assert ik.solve((x, y, 0.24)) is not None
