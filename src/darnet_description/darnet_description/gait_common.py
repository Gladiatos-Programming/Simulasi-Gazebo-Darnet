#!/usr/bin/env python3
"""
Shared multi-step sequence player for ported KRI2023.ino movements that,
unlike the single-shot KudaPose, chain several Leg()/Hand()/Body() + moveOn()
calls in a row (jalanDiTempat, jalan, putarKiri/Kanan, kepitingKanan/Kiri,
tendang, ...). One JointTrajectory point per moveOn() call, published and
waited out in order, mirrors the old firmware's blocking moveOn() loop.
"""
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def make_pose_publisher(node_name):
    rclpy.init()
    node = Node(node_name)
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
    time.sleep(0.5)  # let the publisher match up with the bridge's subscription
    return node, pub


def publish_step(pub, pose, duration_s):
    """Publish one pose dict (partial or full -- any subset of the 20 joint
    names) as a single JointTrajectory point, then block until duration_s
    has elapsed (matching moveOn()'s blocking behavior)."""
    msg = JointTrajectory()
    msg.joint_names = list(pose.keys())
    point = JointTrajectoryPoint()
    point.positions = list(pose.values())
    sec = int(duration_s)
    point.time_from_start = Duration(sec=sec, nanosec=int((duration_s - sec) * 1e9))
    msg.points.append(point)
    pub.publish(msg)
    time.sleep(duration_s)


def run_sequence(pub, steps, cycles=1):
    """steps: list of (pose_dict, duration_s) tuples, played in order.
    cycles: how many times to repeat the whole list (old firmware's loop()
    called these functions repeatedly; a script run once needs an explicit
    repeat count instead)."""
    for _ in range(cycles):
        for pose, duration_s in steps:
            publish_step(pub, pose, duration_s)


def finish(node):
    node.destroy_node()
    rclpy.shutdown()
