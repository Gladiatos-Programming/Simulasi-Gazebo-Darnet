#!/usr/bin/env python3
"""
ROS2 <-> U2D2 bridge for MX-28/MX-64 running Protocol 1.0 firmware.

Unlike the OpenRB-150 (which has its own microcontroller running firmware
that parses the CSV and drives the servos), U2D2 is just a USB<->TTL
converter with no onboard brain. This node does what that firmware used
to do: convert each incoming JointTrajectory point into per-servo
Dynamixel goal-position/moving-speed writes over the bus directly.

These servos turned out to be running Protocol 1.0, not 2.0 -- Wizard's
scan needed "Protocol 1.0" checked to find them. Protocol 1.0 uses the
legacy AX/RX/MX control table, which is a DIFFERENT memory map from
Protocol 2.0's (2-byte fields here, not 4-byte, at different addresses).

STATUS (see the ID/inversion history in git log / conversation for detail):
- ADDR_* are the standard Protocol 1.0 MX-series addresses -- confirmed
  against Dynamixel Wizard's Control Table tab.
- BAUDRATE confirmed against Wizard (1M bps).
- ORDERED_JOINT_NAMES / INVERTED_JOINTS: all 20 joints verified against
  real hardware via CalibrateInversion.py, after fixing a right-leg
  joint-naming inconsistency in darnet.xacro itself (Paha Kanan
  Putar/Paha Atas Kanan were on swapped axes).
- Zero-tick offset per joint: measured via CalibrateZeroOffset.py for all
  20 joints (rough hand-posed accuracy, +/-~9 deg observed spread -- good
  enough to use, not perfect).
- Real per-joint mechanical range limits: NOT yet measured (still TODO).
- Safety net (STARTUP_TORQUE_LIMIT / per-command rate limit) removed for now:
  inversion + zero-offset are confirmed correct on real hardware, and the
  conservative torque/rate limits were themselves causing multi-run
  convergence (a single JointTrajectory point couldn't move far enough in
  one shot to reach a full crouch). Re-introduce a real safety net once
  per-joint mechanical range limits are measured.
"""

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite,
    DXL_LOBYTE, DXL_HIBYTE, COMM_SUCCESS,
)

DEVICE_NAME = '/dev/ttyUSB0'   # U2D2 usually enumerates as ttyUSB0, not ttyACM0
BAUDRATE = 1000000  # confirmed via Dynamixel Wizard control table (Baud Rate = 1)
PROTOCOL_VERSION = 1.0

# Protocol 1.0 legacy control table (AX/RX/MX-series) -- confirm against
# Dynamixel Wizard's Control Table tab, NOT the Protocol 2.0 addresses
# (64/112/116) used by X-series -- those are wrong for this firmware.
ADDR_TORQUE_ENABLE = 24    # 1 byte
ADDR_GOAL_POSITION = 30    # 2 bytes
ADDR_MOVING_SPEED = 32     # 2 bytes, unit ~0.111 rev/min; 0 = max speed (no limit)
ADDR_TORQUE_LIMIT = 34     # 2 bytes, 0-1023

TICKS_PER_REV = 4096
DEFAULT_ZERO_OFFSET_TICKS = 2048
MOVING_SPEED_UNIT_RPM = 0.111
MIN_MOVING_SPEED = 5     # floor so a joint never gets 0 (=max/uncapped speed)
MAX_MOVING_SPEED = 1023  # true max -- no artificial ceiling (see module docstring)

# Safety net disabled for now (see module docstring) -- full torque, no
# per-joint range clamp, no per-command rate limit. Re-introduce once real
# per-joint mechanical limits are measured.
STARTUP_TORQUE_LIMIT = 1023   # full torque (max value for this control table)

# Verified against physical hardware via Dynamixel Wizard + CalibrateInversion.py.
# (The right leg's xacro joint names used to have "Paha Kanan Putar" and
# "Paha Atas Kanan" on the wrong axes -- fixed directly in darnet.xacro so
# the names now match physical reality and the left leg's own convention;
# no ID swap needed here since this order was correct all along.)
ORDERED_JOINT_NAMES = [
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
    'Tangan Kiri', 'Tangan Kanan',
    'Paha Kanan Putar', 'Paha Kiri Putar', 'Paha Atas Kanan', 'Paha Atas Kiri',
    'Paha Bawah Kanan', 'Paha Bawah Kiri', 'Lutut Kanan', 'Lutut Kiri',
    'Kaki Kanan Atas', 'Kaki Kiri Atas', 'Kaki Kanan Bawah', 'Kaki Kiri Bawah',
    'Leher Putar', 'Kepala Putar',
]
DXL_IDS = {name: i + 1 for i, name in enumerate(ORDERED_JOINT_NAMES)}

# Full 20-joint calibration via CalibrateInversion.py, verified against
# physical hardware and the corrected darnet.xacro joint naming.
INVERTED_JOINTS = {
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Tangan Kiri', 'Tangan Kanan',
    'Paha Kanan Putar', 'Paha Kiri Putar', 'Paha Bawah Kanan', 'Paha Bawah Kiri',
    'Lutut Kanan', 'Lutut Kiri', 'Kaki Kanan Bawah', 'Kaki Kiri Bawah', 'Leher Putar',
}


def _load_zero_offsets():
    """Per-joint zero-tick measured by CalibrateZeroOffset.py. Falls back to
    DEFAULT_ZERO_OFFSET_TICKS for any joint not (yet) calibrated."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('darnet_description'), 'config', 'zero_offsets.json')
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {}


ZERO_OFFSETS = _load_zero_offsets()


def rad_to_ticks(name, radians):
    sign = -1.0 if name in INVERTED_JOINTS else 1.0
    zero = ZERO_OFFSETS.get(name, DEFAULT_ZERO_OFFSET_TICKS)
    ticks = zero + int(round(sign * radians * TICKS_PER_REV / (2 * 3.141592653589793)))
    return max(0, min(TICKS_PER_REV - 1, ticks))


def le_bytes_2(value):
    return [DXL_LOBYTE(value), DXL_HIBYTE(value)]


class RosToU2D2(Node):
    def __init__(self):
        super().__init__('ros_to_u2d2_bridge')

        self.port = PortHandler(DEVICE_NAME)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            raise RuntimeError(f'Failed to open {DEVICE_NAME}')
        if not self.port.setBaudRate(BAUDRATE):
            raise RuntimeError(f'Failed to set baud {BAUDRATE}')

        self.last_known_positions = {name: 0.0 for name in ORDERED_JOINT_NAMES}
        self.last_goal_ticks = {
            name: ZERO_OFFSETS.get(name, DEFAULT_ZERO_OFFSET_TICKS) for name in ORDERED_JOINT_NAMES
        }

        for name, dxl_id in DXL_IDS.items():
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_TORQUE_LIMIT, STARTUP_TORQUE_LIMIT)
            result, _ = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f"Torque enable failed for {name} (ID {dxl_id}): "
                    f"{self.packet.getTxRxResult(result)}")

        self.sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback, 10)

        # PinnochioIK.py (and Centerized.py) block/refuse to act without
        # /joint_states -- there's no Gazebo here to provide it, so this
        # echoes last-COMMANDED position, not real servo feedback. Good
        # enough to unblock IK initialization; NOT proprioception -- a
        # stalled/blocked joint will still report as if it reached target.
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        self.publish_joint_states()  # once immediately so IK can init without waiting

        self.get_logger().info(f"U2D2 bridge ready on {DEVICE_NAME} @ {BAUDRATE}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.last_known_positions.keys())
        msg.position = list(self.last_known_positions.values())
        self.joint_state_pub.publish(msg)

    def listener_callback(self, msg):
        if not msg.points:
            return
        point = msg.points[0]
        for name, pos in zip(msg.joint_names, point.positions):
            if name in self.last_known_positions:
                self.last_known_positions[name] = pos

        duration_s = max(point.time_from_start.sec + point.time_from_start.nanosec / 1e9, 0.02)

        speed_write = GroupSyncWrite(self.port, self.packet, ADDR_MOVING_SPEED, 2)
        pos_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)

        for name in ORDERED_JOINT_NAMES:
            dxl_id = DXL_IDS[name]
            goal_ticks = rad_to_ticks(name, self.last_known_positions[name])

            travel_rev = abs(goal_ticks - self.last_goal_ticks[name]) / TICKS_PER_REV
            rpm = travel_rev / (duration_s / 60.0)
            speed_units = int(max(MIN_MOVING_SPEED,
                                   min(MAX_MOVING_SPEED, rpm / MOVING_SPEED_UNIT_RPM)))

            speed_write.addParam(dxl_id, le_bytes_2(speed_units))
            pos_write.addParam(dxl_id, le_bytes_2(goal_ticks))
            self.last_goal_ticks[name] = goal_ticks

        speed_write.txPacket()
        pos_write.txPacket()
        speed_write.clearParam()
        pos_write.clearParam()

    def destroy_node(self):
        for dxl_id in DXL_IDS.values():
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosToU2D2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
