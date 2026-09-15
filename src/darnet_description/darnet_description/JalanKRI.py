#!/usr/bin/env python3
"""
Actual walking gait -- ported from jalan() (+ transisiJalan()/pascaJalan())
in KRI2023.ino. Named JalanKRI, not Jalan, because Jalan.py already exists
in this package and is a completely different, unrelated Pinocchio-IK-based
script (not a port of this firmware) -- left untouched.

Legs + arms each step (the old firmware calls Hand() every jalan() step but
never Body(), so head is left untouched here too -- see
kri_translation.build_leg_arm_pose).

Sequence: one settle step (transisiJalan, legs only) -> N walk cycles (4
steps each, ~0.15s/step) -> one settle step (pascaJalan, legs+arms).

Run with ComsROS2U2D2.py already running:
  ros2 run darnet_description JalanKRI [cycles]
"""
import sys

from darnet_description.kri_translation import (
    leg_ik, hand_angles, build_leg_pose, build_leg_arm_pose, apply_flip_config,
)
from darnet_description.gait_common import make_pose_publisher, run_sequence, finish

STEP_DURATION_S = 0.15
SETTLE_DURATION_S = 0.2
DEFAULT_CYCLES = 4


def compute_transisi_step():
    normal_foot_height, tilt_offset, rotasi_kaki, tegak9 = 190, 15, 0, 7 + 9
    right = leg_ik(0, normal_foot_height - 20, tilt_offset, rotasi_kaki, tegak9, mirror=False)
    left = leg_ik(0, normal_foot_height - 20, -tilt_offset, -rotasi_kaki, tegak9, mirror=True)
    pose = apply_flip_config(build_leg_pose(right, left), verbose=False)
    return (pose, SETTLE_DURATION_S)


def compute_walk_steps():
    walk_distance, walk_distance_offset = 15, -15
    normal_foot_height, tilt_offset, rotasi_kaki10, tegak10 = 190, 15, 10, 7 + 10
    center = 150

    def step(right_x, right_y, right_z, hand_r, left_x, left_y, left_z, hand_l):
        right = leg_ik(right_x, right_y, right_z, rotasi_kaki10, tegak10, mirror=False)
        left = leg_ik(left_x, left_y, left_z, -rotasi_kaki10, tegak10, mirror=True)
        hand_right_old = hand_angles(*hand_r, mirror=False)
        hand_left_old = hand_angles(*hand_l, mirror=True)
        pose = build_leg_arm_pose(right, left, hand_right_old, hand_left_old)
        return apply_flip_config(pose, verbose=False)

    y = normal_foot_height - 20
    steps = [
        step(walk_distance_offset - 10, y, tilt_offset, (180, center, 150),
             walk_distance + 10, y, -tilt_offset, (180, center, 150)),
        step(0, normal_foot_height - 40, tilt_offset, (190, center, 150),
             0, y, -tilt_offset, (170, center, 150)),
        step(walk_distance + 10, y, tilt_offset, (180, center, 150),
             walk_distance_offset - 10, y, -tilt_offset, (180, center, 150)),
        step(0, y, tilt_offset, (170, center, 150),
             0, normal_foot_height - 40, -tilt_offset, (190, center, 150)),
    ]
    return [(pose, STEP_DURATION_S) for pose in steps]


def compute_pasca_step():
    walk_distance, walk_distance_offset = 15, -15
    normal_foot_height, tilt_offset, rotasi_kaki10, tegak10 = 190, 15, 10, 7 + 10
    center = 150
    y = normal_foot_height - 20
    right = leg_ik(walk_distance_offset - 5, y, tilt_offset, rotasi_kaki10, tegak10, mirror=False)
    left = leg_ik(walk_distance + 5, y, -tilt_offset, -rotasi_kaki10, tegak10, mirror=True)
    hand_right_old = hand_angles(180, center, 150, mirror=False)
    hand_left_old = hand_angles(180, center, 150, mirror=True)
    pose = apply_flip_config(build_leg_arm_pose(right, left, hand_right_old, hand_left_old), verbose=False)
    return (pose, SETTLE_DURATION_S)


def main(args=None):
    cycles = DEFAULT_CYCLES
    if len(sys.argv) > 1:
        cycles = int(sys.argv[1])

    walk_steps = compute_walk_steps()
    node, pub = make_pose_publisher('jalan_kri')
    node.get_logger().info(f"JalanKRI: settle -> {cycles} walk cycle(s) -> settle...")
    run_sequence(pub, [compute_transisi_step()])
    run_sequence(pub, walk_steps, cycles=cycles)
    run_sequence(pub, [compute_pasca_step()])
    node.get_logger().info("Done.")
    finish(node)


if __name__ == '__main__':
    main()
