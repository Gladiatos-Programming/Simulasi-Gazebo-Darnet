#!/usr/bin/env python3
"""
"Kuda" (ready/crouch stance) -- ported from kuda(30) in KRI2023.ino /
Kinematics_learn_Darnet_ver_2_fixed, the pose run once at startup on the
competition robot.

See kri_translation.py for the shared conversion pipeline and ID mapping.
kri_translation.EXTRA_FLIP is currently empty (reverted pending a clean
re-test) -- for FAST per-joint iteration without rebuilding, this script
uses kri_translation.apply_flip_config(), which reads kuda_flip_config.json
(repo root, NOT inside the ROS2 package, so editing it never needs
`colcon build`) at every run:
  1  = no flip (base computed value, matches kri_translation's own math)
  -1 = flip that joint's sign
Edit the JSON, save, re-run `ros2 run darnet_description KudaPose` --
that's the whole loop, no rebuild in between. Every other ported movement
script uses the same shared config -- it's a joint-level correction, not
specific to this pose.

Once a flip combination is confirmed correct on real hardware, move it
into kri_translation.py's EXTRA_FLIP (the permanent, shared fix used by
every ported function), not left sitting only in this per-script config.

CAVEAT: leg segment lengths (kri_translation.RIGHT1/RIGHT2/LEFT1/LEFT2 =
93mm) are from the old firmware, not re-measured against the current
robot.

Run with ComsROS2U2D2.py already running:
  ros2 run darnet_description KudaPose
"""
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from darnet_description.kri_translation import (
    leg_ik, hand_angles, build_pose, apply_flip_config,
    RIGHT_LEG_NAMES, LEFT_LEG_NAMES,
)

MOVE_DURATION_S = 1.0  # fast move -- safety net removed, full torque, one-shot reach


def compute_kuda_pose():
    walk_distance_offset = -15
    walk_distance = 15
    normal_foot_height = 190
    a_param = 30  # kuda(30)

    x = walk_distance_offset + walk_distance - 10
    y = normal_foot_height - a_param

    angle_left = leg_ik(x, y, 0, 0, 10, mirror=True)
    angle_right = leg_ik(x, y, 0, 0, 10, mirror=False)
    hand_right_old = hand_angles(150, 150, 150, mirror=False)  # old Hand(150,150,150,0)
    hand_left_old = hand_angles(150, 150, 150, mirror=True)    # old Hand(150,150,150,1)

    pose = build_pose(angle_right, angle_left, hand_right_old, hand_left_old)

    # Sanity check: sign flips (from EXTRA_FLIP or the per-run config below)
    # never change magnitude, so left/right legs should always match here
    # regardless of flip config -- this only catches a genuine geometry bug.
    for r_name, l_name in zip(RIGHT_LEG_NAMES, LEFT_LEG_NAMES):
        if abs(abs(pose[r_name]) - abs(pose[l_name])) > 0.01:
            raise AssertionError(
                f"Symmetry check failed: {r_name}={pose[r_name]:.4f} vs "
                f"{l_name}={pose[l_name]:.4f} -- magnitudes should match.")

    apply_flip_config(pose)
    return pose


def main(args=None):
    pose = compute_kuda_pose()

    rclpy.init(args=args)
    node = Node('kuda_pose')
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
    time.sleep(0.5)

    msg = JointTrajectory()
    msg.joint_names = list(pose.keys())
    point = JointTrajectoryPoint()
    point.positions = list(pose.values())
    sec = int(MOVE_DURATION_S)
    point.time_from_start = Duration(sec=sec, nanosec=int((MOVE_DURATION_S - sec) * 1e9))
    msg.points.append(point)

    node.get_logger().info(f"Moving to kuda (ready stance) over {MOVE_DURATION_S}s...")
    pub.publish(msg)
    time.sleep(MOVE_DURATION_S + 1.0)
    node.get_logger().info("Done.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
