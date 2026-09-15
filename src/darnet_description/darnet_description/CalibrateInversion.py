#!/usr/bin/env python3
"""
Interactive per-joint inversion calibration for the real Darnet robot.

Talks to the servos DIRECTLY via dynamixel_sdk (Protocol 1.0), bypassing
the ROS2 bridge entirely -- this works even before ComsROS2U2D2.py's ID
mapping and INVERTED_JOINTS set are fully trusted, which is the point.

HOW TO USE:
  1. Run this script. For each joint it opens a reference image
     (calibration_refs/<Joint_Name>.png) showing the robot at rest next to
     the robot with that one joint rotated positive (highlighted red) --
     generated directly from the URDF/MuJoCo model, so it's ground truth
     for what "positive" means in the same coordinate convention the
     bridge uses.
  2. Look at the image, then press Enter; it moves the real servo a small
     amount, slowly, at reduced torque. Watch the real robot.
  3. Answer whether the real joint moved the same way as the image showed.
  4. At the end it prints which joints (if any) need INVERTED_JOINTS
     flipped in ComsROS2U2D2.py.

(Interactive MuJoCo-viewer joint dragging was tried first and dropped --
too unreliable across viewer/window-manager combinations. Static reference
images generated per joint are more robust.)

Run one joint at a time by passing joint names as arguments, e.g.:
  python3 CalibrateInversion.py "Lengan Kiri" "Lengan Kanan"
With no arguments it walks through all 20 in order.

SAFETY: put the robot in a relaxed, obstruction-free pose (stand/gantry
ideally) before running this -- per-joint real range limits are not yet
verified, and this uses a fixed small delta (~13 deg) from whatever the
current position happens to be. Reduced torque (about 40%) is set for the
duration of the test so a blocked joint stalls softly instead of forcing
through.
"""

import os
import subprocess
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


def _find_refs_dir():
    # Prefer the installed share dir (correct when run via `ros2 run`, since
    # __file__ then points into install/.../site-packages, not the source
    # tree). Fall back to the source-relative path for `python3 CalibrateInversion.py`.
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(get_package_share_directory('darnet_description'), 'calibration_refs')
        if os.path.isdir(installed):
            return installed
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'calibration_refs')


REFS_DIR = _find_refs_dir()

_VIEWER_CMD = None
for _candidate in (['eog', '--new-instance'], ['xdg-open']):
    from shutil import which as _which
    if _which(_candidate[0]):
        _VIEWER_CMD = _candidate
        break

_current_viewer_proc = None


def open_reference_image(name):
    """Opens the reference image in its own fresh viewer process, killing
    whatever was open before -- xdg-open/eog often reuse an existing window
    instead of loading the new file into it, which silently shows a STALE
    image from an earlier joint. Killing the old process before opening the
    new one guarantees there's nothing stale left to look at."""
    global _current_viewer_proc

    if _current_viewer_proc is not None:
        try:
            _current_viewer_proc.terminate()
            _current_viewer_proc.wait(timeout=1.0)
        except Exception:
            pass
        _current_viewer_proc = None

    path = os.path.join(REFS_DIR, name.replace(' ', '_') + '.png')
    if not os.path.exists(path):
        print(f"(no reference image found at {path})")
        return
    if _VIEWER_CMD is None:
        print(f"(no image viewer found -- open manually: {path})")
        return
    try:
        _current_viewer_proc = subprocess.Popen(
            _VIEWER_CMD + [path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        print(f"Reference image: {path}")


def close_reference_image():
    global _current_viewer_proc
    if _current_viewer_proc is not None:
        try:
            _current_viewer_proc.terminate()
        except Exception:
            pass
        _current_viewer_proc = None

DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

# Protocol 1.0 legacy control table -- same addresses as ComsROS2U2D2.py
ADDR_TORQUE_ENABLE = 24     # 1 byte
ADDR_GOAL_POSITION = 30     # 2 bytes
ADDR_MOVING_SPEED = 32      # 2 bytes
ADDR_TORQUE_LIMIT = 34      # 2 bytes
ADDR_PRESENT_POSITION = 36  # 2 bytes

TICKS_PER_REV = 4096
TEST_DELTA_TICKS = 150   # ~13 degrees -- small, clearly visible, low risk
TEST_SPEED = 40          # slow
TEST_TORQUE_LIMIT = 400  # ~40% of max (1023) -- reduced for calibration safety
SETTLE_TIME_S = 1.5

# Same wiring order as ComsROS2U2D2.py -- see the comment there for why
# "Paha Kanan Putar"/"Paha Atas Kanan" ended up on the axes they're on
# (fixed in darnet.xacro itself, not here).
ORDERED_JOINT_NAMES = [
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
    'Tangan Kiri', 'Tangan Kanan',
    'Paha Kanan Putar', 'Paha Kiri Putar', 'Paha Atas Kanan', 'Paha Atas Kiri',
    'Paha Bawah Kanan', 'Paha Bawah Kiri', 'Lutut Kanan', 'Lutut Kiri',
    'Kaki Kanan Atas', 'Kaki Kiri Atas', 'Kaki Kanan Bawah', 'Kaki Kiri Bawah',
    'Leher Putar', 'Kepala Putar',
]
DXL_IDS = {name: i + 1 for i, name in enumerate(ORDERED_JOINT_NAMES)}

# What ComsROS2U2D2.py currently has -- used only to flag mismatches at the end
CURRENT_INVERTED = {
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Tangan Kiri', 'Tangan Kanan',
    'Paha Kanan Putar', 'Paha Kiri Putar', 'Paha Bawah Kanan', 'Paha Bawah Kiri',
    'Lutut Kanan', 'Lutut Kiri', 'Kaki Kanan Bawah', 'Kaki Kiri Bawah', 'Leher Putar',
}


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
    print("INVERSION CALIBRATION")
    print("For each joint: a reference image opens (left=neutral, right=that")
    print("joint rotated positive, highlighted red), generated from the URDF/")
    print("MuJoCo model. Then this moves the real servo the same way -- check")
    print("whether it matches.")
    print("Type 'q' at any prompt to stop early (torque is disabled on exit).")
    print("=" * 70)

    results = {}

    for name in order:
        dxl_id = DXL_IDS[name]

        packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, TEST_TORQUE_LIMIT)
        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

        pos, res, err = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS:
            print(f"\n[{name}] ID {dxl_id}: no response ({packet.getTxRxResult(res)}) -- skipping")
            results[name] = 'no_response'
            continue

        print(f"\n--- {name} (ID {dxl_id}) --- current tick: {pos}")
        open_reference_image(name)
        ans = input(f"Reference image opened (left=neutral, right=positive, "
                     f"joint highlighted red). Look at it, then press Enter "
                     f"to move the real servo (or 'q' to quit): ").strip().lower()
        if ans == 'q':
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            close_reference_image()
            print("Stopping early.")
            break

        target = max(0, min(TICKS_PER_REV - 1, pos + TEST_DELTA_TICKS))
        packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, TEST_SPEED)
        packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, target)
        time.sleep(SETTLE_TIME_S)

        ans = input(f"Did {name} move the SAME direction as shown in the reference image? "
                     f"[y]es / [n]o (opposite) / [s]kip / [q]uit: ").strip().lower()

        packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, pos)
        time.sleep(SETTLE_TIME_S)
        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        if ans == 'q':
            close_reference_image()
            print("Stopping early.")
            break
        elif ans == 'y':
            results[name] = 'match'
        elif ans == 'n':
            results[name] = 'flip'
        else:
            results[name] = 'skip'

    close_reference_image()
    port.closePort()

    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    should_invert = set()
    for name, r in results.items():
        currently = name in CURRENT_INVERTED
        if r == 'match':
            needed = False
        elif r == 'flip':
            needed = True
        else:
            print(f"{name:20s}  {r} -- unresolved, left as-is "
                  f"(currently {'inverted' if currently else 'not inverted'})")
            continue
        if needed:
            should_invert.add(name)
        mismatch = "  <-- MISMATCH vs current INVERTED_JOINTS" if needed != currently else ""
        print(f"{name:20s}  should be {'INVERTED' if needed else 'not inverted'} "
              f"(currently {'inverted' if currently else 'not inverted'}){mismatch}")

    print("\nIf any mismatches were flagged, update INVERTED_JOINTS in "
          "ComsROS2U2D2.py to include exactly these names:")
    print(sorted(should_invert))


if __name__ == '__main__':
    main()
