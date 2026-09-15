#!/usr/bin/env python3
"""
Live jog-and-lock trimmer that starts from a POSE's commanded position
(currently: KudaPose) instead of the zero-offset/neutral position.

Why: TrimZeroOffset.py starts at the straight-leg zero pose, but the thing
you're actually trying to fix (e.g. a knee looking too bent) shows up in a
DIFFERENT pose (the crouch). Jogging from the actual pose you're looking
at is more direct than trimming zero and hoping it transfers correctly.

It still ends up as a permanent zero-offset fix, not a one-off patch:
since ticks = zero_offset + signed_radians (see ComsROS2U2D2.rad_to_ticks),
nudging the final commanded tick by some delta is mathematically the same
as adding that delta to the zero-offset -- so whatever you confirm here
gets applied to zero_offsets.json directly, affecting every pose that
joint is ever commanded to, not just this one.

Usage:
  ros2 run darnet_description TrimFromPose "Lutut Kiri"
"""
import json
import os
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler

from darnet_description.KudaPose import compute_kuda_pose
from darnet_description.ComsROS2U2D2 import rad_to_ticks, DXL_IDS

DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34

TICKS_PER_REV = 4096
TRIM_SPEED = 30
TRIM_TORQUE_LIMIT = 300  # ~30%, gentle


def _find_share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = get_package_share_directory('darnet_description')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


ZERO_OFFSETS_PATH = os.path.join(_find_share_dir(), 'config', 'zero_offsets.json')


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in DXL_IDS:
        print("Usage: TrimFromPose <joint name>")
        print("Joint names:", ", ".join(DXL_IDS.keys()))
        sys.exit(1)

    name = sys.argv[1]
    dxl_id = DXL_IDS[name]

    pose = compute_kuda_pose()
    starting_tick = rad_to_ticks(name, pose[name])

    try:
        with open(ZERO_OFFSETS_PATH) as f:
            offsets = json.load(f)
    except Exception:
        offsets = {}
    old_zero = offsets.get(name, 2048)

    port = PortHandler(DEVICE_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        print(f"Failed to open {DEVICE_NAME}")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print(f"Failed to set baud {BAUDRATE}")
        sys.exit(1)

    packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, TRIM_TORQUE_LIMIT)
    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
    packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, TRIM_SPEED)

    print("=" * 70)
    print(f"TRIMMING FROM KUDA POSE: {name} (ID {dxl_id})")
    print(f"KudaPose commands this joint to {pose[name]:+.4f} rad, "
          f"which is currently tick {starting_tick} (zero-offset {old_zero})")
    print("NOTE: this only moves this one joint -- the rest of the robot is")
    print("not being posed, so this checks THIS joint's angle in isolation,")
    print("not the whole crouch. Fine for a symmetric single-joint check.")
    print("=" * 70)

    current = starting_tick
    packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, current)
    time.sleep(1.0)

    while True:
        cmd = input(f"\n[{name}] at tick {current} (kuda target was {starting_tick}). "
                     f"Enter a nudge (e.g. 15 or -15), 'y' to save as new zero-offset, "
                     f"'q' to quit without saving: ").strip()
        if cmd.lower() == 'y':
            delta = current - starting_tick
            new_zero = max(0, min(TICKS_PER_REV - 1, old_zero + delta))
            offsets[name] = new_zero
            os.makedirs(os.path.dirname(ZERO_OFFSETS_PATH), exist_ok=True)
            with open(ZERO_OFFSETS_PATH, 'w') as f:
                json.dump(offsets, f, indent=2, sort_keys=True)
            print(f"Nudge was {delta:+d} ticks -> zero-offset updated: "
                  f"{old_zero} -> {new_zero}. Saved to {ZERO_OFFSETS_PATH}")
            break
        if cmd.lower() == 'q':
            print("Quit without saving -- zero_offsets.json unchanged.")
            break
        try:
            delta_input = int(cmd)
        except ValueError:
            print("Enter a signed integer tick delta, 'y', or 'q'.")
            continue
        current = max(0, min(TICKS_PER_REV - 1, current + delta_input))
        packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, current)
        time.sleep(0.3)

    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    port.closePort()


if __name__ == '__main__':
    main()
