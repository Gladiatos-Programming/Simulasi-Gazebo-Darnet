#!/usr/bin/env python3
"""
Smallest possible real-hardware motion test: moves ONE joint by a small,
slow, one-shot amount through the ACTUAL pipeline (this publishes a real
JointTrajectory to /joint_trajectory_controller/joint_trajectory, same as
Bangkit.py/Jalan.py/etc. would), then returns it to zero and exits.

Unlike CalibrateInversion.py (which talks to servos directly, bypassing
ComsROS2U2D2.py entirely), this is meant to validate the bridge itself --
ID mapping, inversion, zero-offset, the new rate limit and safe-range clamp
-- all together, the way a real motion script actually uses it.

Run with ComsROS2U2D2.py already running in another terminal:
  ros2 run darnet_description MinimalJointTest "Lengan Kiri" 0.3

Args: <joint name> [radians, default 0.3 ~= 17 deg] [duration seconds, default 2.0]
"""

import sys
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main(args=None):
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    joint_name = sys.argv[1]
    target_rad = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    duration_s = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    rclpy.init(args=args)
    node = Node('minimal_joint_test')
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    time.sleep(0.5)  # let the publisher connect before the first message

    def send(position, label):
        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        sec = int(duration_s)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_s - sec) * 1e9))
        msg.points.append(point)
        pub.publish(msg)
        node.get_logger().info(f"{label}: {joint_name} -> {position:.3f} rad over {duration_s}s")

    node.get_logger().info(f"Moving {joint_name} to +{target_rad} rad, then back to 0. "
                            f"Ctrl+C to abort (won't auto-return to zero if you do).")
    send(target_rad, "OUT")
    time.sleep(duration_s + 0.5)
    send(0.0, "BACK")
    time.sleep(duration_s + 0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
