#!/usr/bin/env python3
"""
"Diam di tempat" (idle/ready stance while scanning for the ball) -- ported
from diamDiTempat() in KRI2023.ino.

Single-shot pose (legs + hands + head all touched at once), same shape as
KudaPose.py. See kri_translation.py for the shared conversion pipeline.

NOTE ON DURATION: the original diamDiTempat() used moveOn(delayTime) i.e.
just 100ms, but that's safe there because in the firmware's control loop
it's always called right after another Leg()/moveOn() sequence that already
left the robot close to this pose. Run standalone via `ros2 run`, the
robot's actual starting pose is unknown, so this uses a longer duration
(like kuda(30)'s startup moveOn(1000)) for a safe one-shot move instead.

Run with ComsROS2U2D2.py already running:
  ros2 run darnet_description DiamDiTempat
"""
from darnet_description.kri_translation import (
    leg_ik, hand_angles, build_pose, apply_flip_config,
    RIGHT_LEG_NAMES, LEFT_LEG_NAMES,
)
from darnet_description.gait_common import make_pose_publisher, publish_step, finish

MOVE_DURATION_S = 1.0


def compute_diam_pose():
    normal_foot_height = 190
    tilt_offset = 15
    rotasi_kaki = 0
    tegak = 7
    center = 150

    angle_left = leg_ik(0, normal_foot_height - 20, -tilt_offset, -rotasi_kaki, tegak, mirror=True)
    angle_right = leg_ik(0, normal_foot_height - 20, tilt_offset, rotasi_kaki, tegak, mirror=False)
    hand_right_old = hand_angles(180, center, 230, mirror=False)  # old Hand(180,center,230,0)
    hand_left_old = hand_angles(180, center, 230, mirror=True)    # old Hand(180,center,230,1)

    pose = build_pose(angle_right, angle_left, hand_right_old, hand_left_old, leher=180, kepala=195)

    for r_name, l_name in zip(RIGHT_LEG_NAMES, LEFT_LEG_NAMES):
        if abs(abs(pose[r_name]) - abs(pose[l_name])) > 0.01:
            raise AssertionError(
                f"Symmetry check failed: {r_name}={pose[r_name]:.4f} vs "
                f"{l_name}={pose[l_name]:.4f} -- magnitudes should match.")

    apply_flip_config(pose)
    return pose


def main(args=None):
    pose = compute_diam_pose()
    node, pub = make_pose_publisher('diam_di_tempat')
    node.get_logger().info(f"Moving to diam di tempat over {MOVE_DURATION_S}s...")
    publish_step(pub, pose, MOVE_DURATION_S)
    node.get_logger().info("Done.")
    finish(node)


if __name__ == '__main__':
    main()
