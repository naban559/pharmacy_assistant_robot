"""Compatibility entry point for the shared, MoveIt-validated commander.

Both executable names use the same TCP, IK, Cartesian descent, collision
checks and feedback verification. MoveIt must be running for either name.
"""
from k12_pick_place.moveit_pick_place_commander import MoveItPickPlaceCommander, main

PickPlaceCommander = MoveItPickPlaceCommander

if __name__ == '__main__':
    main()
