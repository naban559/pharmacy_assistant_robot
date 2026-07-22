from k12_pick_place.amms_task_manager import mission_state_sequence


def test_mission_sequence_requires_perception_before_pick():
    states = mission_state_sequence()

    assert states.index('SEARCH_FOR_OBJECT') < states.index('PICK_OBJECT')
    assert states.index('VALIDATE_OBJECT') < states.index('PICK_OBJECT')
    assert states.index('CONFIRM_FRESH_DETECTION') < states.index('PICK_OBJECT')
    assert states.index('PICK_OBJECT') < states.index('POST_PICK_BACKUP')
    assert states.index('PICK_OBJECT') < states.index('NAVIGATE_TO_PLACE_ZONE')
    assert 'CLEAR_PICK_ZONE' not in states
    assert states.index('POST_PICK_BACKUP') < states.index('NAVIGATE_TO_PLACE_APPROACH')
    assert states.index('NAVIGATE_TO_PLACE_APPROACH') < states.index('NAVIGATE_TO_PLACE_ZONE')


def test_mission_sequence_repositions_before_pick_and_place():
    states = mission_state_sequence()

    assert states.index('REPOSITION_FOR_REACH') < states.index('CONFIRM_FRESH_DETECTION')
    assert states.index('REPOSITION_FOR_PLACE') < states.index('PLACE_OBJECT')


def test_mission_sequence_stages_before_straight_place_approach():
    states = mission_state_sequence()

    assert states.index('NAVIGATE_TO_PLACE_ZONE') < states.index('ALIGN_TO_PLACE_STATION')
    assert states.index('ALIGN_TO_PLACE_STATION') < states.index('STAGE_FOR_PLACE_APPROACH')
    assert states.index('STAGE_FOR_PLACE_APPROACH') < states.index('FINAL_APPROACH_TO_PLACE')
    assert states.index('FINAL_APPROACH_TO_PLACE') < states.index('DETECT_PLACEMENT_SURFACE')
    assert states.index('FINAL_APPROACH_TO_PLACE') < states.index('PLACE_OBJECT')


def test_mission_sequence_separates_base_and_arm_motion():
    states = mission_state_sequence()

    assert states.index('NAVIGATE_TO_PICK_ZONE') < states.index('PICK_OBJECT')
    assert states.index('RETRACT_ARM') < states.index('RETURN_TO_SEARCH_AREA')


def test_mission_sequence_includes_optional_clear_zone():
    states = mission_state_sequence('place_clear')

    assert 'CLEAR_PLACE_ZONE' in states
    assert states.index('CLEAR_PLACE_ZONE') < states.index('RETURN_TO_SEARCH_AREA')


def test_mission_sequence_can_disable_pick_clear_zone():
    states = mission_state_sequence(post_pick_clear_waypoint='')

    assert 'CLEAR_PICK_ZONE' not in states
    assert states.index('POST_PICK_BACKUP') < states.index('NAVIGATE_TO_PLACE_ZONE')
    assert states.index('PICK_OBJECT') < states.index('NAVIGATE_TO_PLACE_ZONE')


def test_mission_sequence_can_enable_pick_clear_zone():
    states = mission_state_sequence(post_pick_clear_waypoint='pick_clear')

    assert 'CLEAR_PICK_ZONE' in states
    assert states.index('POST_PICK_BACKUP') < states.index('CLEAR_PICK_ZONE')
    assert states.index('CLEAR_PICK_ZONE') < states.index('NAVIGATE_TO_PLACE_APPROACH')


def test_mission_sequence_can_disable_post_pick_backup():
    states = mission_state_sequence(post_pick_backup_enabled=False)

    assert 'POST_PICK_BACKUP' not in states
    assert states.index('PICK_OBJECT') < states.index('NAVIGATE_TO_PLACE_APPROACH')


def test_mission_sequence_can_disable_pre_place_route():
    states = mission_state_sequence(pre_place_route_waypoint='')

    assert 'NAVIGATE_TO_PLACE_APPROACH' not in states
    assert states.index('POST_PICK_BACKUP') < states.index('NAVIGATE_TO_PLACE_ZONE')
