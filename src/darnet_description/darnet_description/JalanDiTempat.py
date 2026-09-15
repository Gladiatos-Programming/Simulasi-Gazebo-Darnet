#!/usr/bin/env python3
"""
"Jalan di tempat" (marching in place) -- ported from jalanDiTempat() in
KRI2023.ino. Legs only -- the old firmware never touches Hand()/Body() in
this function, so each step here publishes only the 12 leg joint names,
leaving hands/head at whatever they were last commanded to (see
kri_translation.build_leg_pose's docstring).

One 4-step cycle is ~0.44s (matching the original's 4x moveOn(delayTime+10)
= 4x110ms); the original's main loop ran this repeatedly for ~1.2s
(periodJalanDiTempat) between other moves, so this script defaults to 3
cycles (~1.3s) and takes an optional cycle count.

Run with ComsROS2U2D2.py already running:
  ros2 run darnet_description JalanDiTempat [cycles]
"""
import sys

from darnet_description.kri_translation import (
    leg_ik, build_leg_pose, apply_flip_config,
)
from darnet_description.gait_common import make_pose_publisher, run_sequence, finish

STEP_DURATION_S = 0.11
DEFAULT_CYCLES = 3


def compute_steps():
    normal_foot_height = 190
    tilt_offset = 15
    rotasi_kaki = 0
    tegak2 = 7 + 2  # tegak+2 in the original

    def leg_pose(right_y, right_z, left_y, left_z):
        right = leg_ik(0, right_y, right_z, rotasi_kaki, tegak2, mirror=False)
        left = leg_ik(0, left_y, left_z, -rotasi_kaki, tegak2, mirror=True)
        return apply_flip_config(build_leg_pose(right, left), verbose=False)

    steps = [
        leg_pose(normal_foot_height - 40, tilt_offset - 5, normal_foot_height - 20, -tilt_offset + 5),
        leg_pose(normal_foot_height - 20, tilt_offset - 5, normal_foot_height - 20, -tilt_offset + 5),
        leg_pose(normal_foot_height - 20, tilt_offset - 5, normal_foot_height - 40, -tilt_offset + 5),
        leg_pose(normal_foot_height - 20, tilt_offset - 5, normal_foot_height - 20, -tilt_offset + 5),
    ]
    return [(pose, STEP_DURATION_S) for pose in steps]


def main(args=None):
    cycles = DEFAULT_CYCLES
    if len(sys.argv) > 1:
        cycles = int(sys.argv[1])

    steps = compute_steps()
    node, pub = make_pose_publisher('jalan_di_tempat')
    node.get_logger().info(f"Jalan di tempat: {cycles} cycle(s) of {len(steps)} steps...")
    run_sequence(pub, steps, cycles=cycles)
    node.get_logger().info("Done.")
    finish(node)


if __name__ == '__main__':
    main()
