#!/usr/bin/env python3
"""
Interactive, live jog-and-lock zero-offset trimmer.

CalibrateZeroOffset.py poses the WHOLE robot by eye once and reads all 20
ticks in one shot -- fast, but imprecise for anything hard to judge by eye
(a knee's exact "straight" point, for instance). This tool is the opposite:
ONE joint at a time, torque on (gently), moved live to its current
zero-offset tick, then you nudge it in small steps while watching the real
joint and only save once it's actually confirmed correct.

Usage:
  ros2 run darnet_description TrimZeroOffset "Lutut Kiri"

At the prompt: enter a signed tick delta (e.g. 15 or -15) to nudge and
re-check, 'y' to save the current position as that joint's new zero-offset,
or 'q' to quit without saving.
"""
import json
import os
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34

TICKS_PER_REV = 4096
TRIM_SPEED = 30          # slow, deliberate jog
TRIM_TORQUE_LIMIT = 300  # ~30%, gentle -- this is fine-adjustment, not a big move

ORDERED_JOINT_NAMES = [
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
    'Tangan Kiri', 'Tangan Kanan',
    'Paha Kanan Putar', 'Paha Kiri Putar', 'Paha Atas Kanan', 'Paha Atas Kiri',
    'Paha Bawah Kanan', 'Paha Bawah Kiri', 'Lutut Kanan', 'Lutut Kiri',
    'Kaki Kanan Atas', 'Kaki Kiri Atas', 'Kaki Kanan Bawah', 'Kaki Kiri Bawah',
    'Leher Putar', 'Kepala Putar',
]
DXL_IDS = {name: i + 1 for i, name in enumerate(ORDERED_JOINT_NAMES)}


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
        print("Usage: TrimZeroOffset <joint name>")
        print("Joint names:", ", ".join(ORDERED_JOINT_NAMES))
        sys.exit(1)

    name = sys.argv[1]
    dxl_id = DXL_IDS[name]

    try:
        with open(ZERO_OFFSETS_PATH) as f:
            offsets = json.load(f)
    except Exception:
        offsets = {}
    current = offsets.get(name, 2048)

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
    print(f"TRIMMING: {name} (ID {dxl_id})")
    print(f"Current zero-offset tick: {current} (deviation from default 2048: {current - 2048:+d})")
    print("Moving there now. Watch the real joint.")
    print("=" * 70)

    packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, current)
    time.sleep(1.0)

    while True:
        cmd = input(f"\n[{name}] at tick {current}. Enter a nudge (e.g. 15 or -15), "
                     f"'y' to save this as the new zero-offset, 'q' to quit without saving: ").strip()
        if cmd.lower() == 'y':
            offsets[name] = current
            os.makedirs(os.path.dirname(ZERO_OFFSETS_PATH), exist_ok=True)
            with open(ZERO_OFFSETS_PATH, 'w') as f:
                json.dump(offsets, f, indent=2, sort_keys=True)
            print(f"Saved {name} = {current} to {ZERO_OFFSETS_PATH}")
            break
        if cmd.lower() == 'q':
            print("Quit without saving -- zero_offsets.json unchanged.")
            break
        try:
            delta = int(cmd)
        except ValueError:
            print("Enter a signed integer tick delta, 'y', or 'q'.")
            continue
        current = max(0, min(TICKS_PER_REV - 1, current + delta))
        packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, current)
        time.sleep(0.3)

    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    port.closePort()


if __name__ == '__main__':
    main()
