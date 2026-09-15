#!/usr/bin/env python3
"""
Per-joint zero-offset calibration for the real Darnet robot.

Each Dynamixel servo has its own absolute encoder reading 0-4095 ticks
around one full revolution, with no idea what that means to the robot.
ComsROS2U2D2.py assumes tick 2048 = the URDF's "0 radians" pose for every
servo, but that's only true if every horn was mounted at exactly the same
rotational offset during assembly -- rarely true by hand. This script
measures the REAL zero-tick per servo instead of assuming 2048 for all.

HOW TO USE:
  1. Run this script. It disables torque on all 20 servos immediately (so
     you can freely move the robot by hand) and opens a full-body reference
     image (calibration_refs/ZERO_POSE_FULL_BODY.png) showing the URDF's
     zero pose.
  2. Physically pose the real robot to match that image as closely as you
     can (arms/legs/head all at the neutral position shown).
  3. Press Enter. It reads each servo's current tick -- no motion, no
     torque, nothing commanded. Purely passive.
  4. Results are saved to config/zero_offsets.json, which ComsROS2U2D2.py
     will pick up automatically. Joints whose measured tick differs a lot
     from the default 2048 are flagged so you can double-check the robot
     was actually posed correctly for that joint before trusting it.

Run a subset by passing joint names as arguments, same as
CalibrateInversion.py. Results merge into the existing file rather than
overwriting it, so you can do this incrementally.
"""

import json
import os
import subprocess
import sys

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24     # 1 byte
ADDR_PRESENT_POSITION = 36  # 2 bytes

TICKS_PER_REV = 4096
DEFAULT_ZERO_TICKS = 2048
FLAG_DEVIATION_TICKS = 600  # ~52 degrees -- worth a second look, not necessarily wrong

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


SHARE_DIR = _find_share_dir()
REF_IMAGE = os.path.join(SHARE_DIR, 'calibration_refs', 'ZERO_POSE_FULL_BODY.png')

# Written to the INSTALLED share dir -- same place ComsROS2U2D2.py reads
# from via get_package_share_directory, so results are usable immediately,
# no rebuild needed. (An earlier version of this tried to write back to the
# source tree instead, using __file__, but __file__ points into install/
# when run via `ros2 run`, not the source tree -- same class of bug already
# hit and fixed once in CalibrateInversion.py's reference-image path.)
OUTPUT_PATH = os.path.join(SHARE_DIR, 'config', 'zero_offsets.json')


def open_reference_image():
    if not os.path.exists(REF_IMAGE):
        print(f"(no reference image found at {REF_IMAGE})")
        return
    from shutil import which
    for cmd in (['eog', '--new-instance'], ['xdg-open']):
        if which(cmd[0]):
            try:
                subprocess.Popen(cmd + [REF_IMAGE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return
            except Exception:
                pass
    print(f"Reference image: {REF_IMAGE}")


def load_existing():
    if os.path.exists(OUTPUT_PATH):
        try:
            with open(OUTPUT_PATH) as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def main():
    order = [j for j in ORDERED_JOINT_NAMES if j in sys.argv[1:]] if len(sys.argv) > 1 else ORDERED_JOINT_NAMES

    port = PortHandler(DEVICE_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        print(f"Failed to open {DEVICE_NAME}")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print(f"Failed to set baud {BAUDRATE}")
        sys.exit(1)

    print("=" * 70)
    print("ZERO-OFFSET CALIBRATION")
    print("Disabling torque on all 20 servos so the robot can be posed by hand.")
    print("=" * 70)

    for name, dxl_id in DXL_IDS.items():
        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

    open_reference_image()
    input("\nPose the real robot to match the reference image (all joints at "
          "their neutral/zero position), then press Enter to read positions: ")

    results = load_existing()
    print()
    for name in order:
        dxl_id = DXL_IDS[name]
        pos, res, err = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS:
            print(f"{name:20s} ID {dxl_id}: no response ({packet.getTxRxResult(res)}) -- skipping")
            continue

        results[name] = pos
        deviation = abs(pos - DEFAULT_ZERO_TICKS)
        flag = "  <-- far from default 2048, double-check this joint was posed correctly" \
            if deviation > FLAG_DEVIATION_TICKS else ""
        print(f"{name:20s} ID {dxl_id}: zero tick = {pos} (deviation from default: {pos - DEFAULT_ZERO_TICKS:+d}){flag}")

    port.closePort()

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, 'w') as f:
        json.dump(results, f, indent=2, sort_keys=True)

    print(f"\nSaved {len(results)} joint(s) to {OUTPUT_PATH}")
    print("ComsROS2U2D2.py reads from this same installed path, so it picks "
          "this up immediately -- no rebuild needed. Missing joints fall "
          "back to the default 2048. (Worth copying this file back into "
          "src/darnet_description/config/ and committing it, though, so a "
          "clean rebuild doesn't lose it.)")


if __name__ == '__main__':
    main()
