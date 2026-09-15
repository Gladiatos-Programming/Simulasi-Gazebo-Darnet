#!/usr/bin/env python3
"""
Shared translation layer for porting KRI2023.ino / Kinematics_learn_Darnet_
ver_2_fixed functions to our ROS2/joint-name architecture. Used by
KudaPose.py and every future ported function (jalan, putarKiri/Kanan,
kepitingKanan/Kiri, tendang, diamDiTempat, ...) -- keep this the single
source of truth for the conversion rather than re-deriving it per script.

=== ID mapping (verified this session, see compare_id_mapping.py) ===
- Legs (old IDs 7-18): direct positional match with our joint order, no
  side swap needed.
- Arms (old IDs 1-6): SIDE-SWAPPED relative to our names -- old "Kanan" =
  our "Kiri" and vice versa. hand_right_old(...) values go to our KIRI
  joints, hand_left_old(...) values go to our KANAN joints.
- Neck/head (19, 20): direct match, no side to confuse.

=== Two separate inversion layers -- do not conflate them ===
1. ComsROS2U2D2.py's own INVERTED_JOINTS: independently calibrated against
   real hardware via CalibrateInversion.py. Applies automatically to
   whatever radians this module computes. NEVER touch that set to fix a
   translation bug -- it's correct for every other script that already
   uses it.
2. EXTRA_FLIP below: found empirically by running KudaPose.py on the real
   robot (v1/v2 of that script got this wrong -- see its docstring for the
   v1->v2 bugfix history). This is a SEPARATE correction needed only for
   angles computed via THIS translation pipeline, on top of layer 1 above.
   Confirmed correct joints (left leg after the mirror-conversion fix,
   Bahu Tangan Kanan/Kiri, Tangan Kanan, neck/head) are NOT in this set --
   don't add speculatively, only add what's been confirmed wrong on
   hardware.
"""
import json
import math
import os

PHI = 57.295779513082320876798154814105  # 180/pi, matches the .ino exactly

RIGHT1 = LEFT1 = 128.0  # mm, hip-to-knee, measured from darnet_mujoco.xml joint geometry
RIGHT2 = LEFT2 = 121.0  # mm, knee-to-ankle, measured from darnet_mujoco.xml joint geometry
# (was 93.0/93.0, an old-firmware value never re-verified against this robot --
# that ~30-38% length error was throwing off the leg IK's foot placement badly
# enough to leave KudaPose's torso settling at a stable ~20deg tilt in sim
# instead of upright, confirmed independent of actuator gain -- see SimBridge
# physics debugging session.)

RIGHT_LEG_NAMES = ['Paha Kanan Putar', 'Paha Atas Kanan', 'Paha Bawah Kanan',
                    'Lutut Kanan', 'Kaki Kanan Atas', 'Kaki Kanan Bawah']
LEFT_LEG_NAMES = ['Paha Kiri Putar', 'Paha Atas Kiri', 'Paha Bawah Kiri',
                   'Lutut Kiri', 'Kaki Kiri Atas', 'Kaki Kiri Bawah']
OUR_KIRI_ARM_NAMES = ['Lengan Kiri', 'Bahu Tangan Kiri', 'Tangan Kiri']
OUR_KANAN_ARM_NAMES = ['Lengan Kanan', 'Bahu Tangan Kanan', 'Tangan Kanan']

# REVERTED (2026-09-12): the previous EXTRA_FLIP set (right leg entirely +
# Lengan Kanan/Kiri + Tangan Kiri) was based on real-hardware feedback that
# turned out to be confounded -- the robot's starting pose likely differed
# between test runs (absolute JointTrajectory targets don't control the
# PATH taken to reach them, only the destination), making it hard to tell
# which joint's target was actually wrong vs. which just moved through an
# odd-looking path. Reverted to empty pending a cleaner re-test from a
# known, consistent starting pose. Do not re-add entries here without
# confirming the robot's starting position first.
EXTRA_FLIP = set()


def leg_ik(x, y, z, angle, setA2, mirror):
    """Direct transcription of Leg() from the .ino. mirror=False -> right,
    True -> left. Returns a[0..6] in the old firmware's final convention
    (already +180 shifted, degrees, 180=center)."""
    temp_angle = angle - 180
    if mirror:
        z = -z
        angle = -angle
        temp_angle = -temp_angle

    r0 = math.sqrt(z * z + x * x)
    B = math.atan2(-z, -x) * PHI - temp_angle
    aX = r0 * math.cos(B / PHI)
    aZ = r0 * math.sin(B / PHI)

    r1 = math.sqrt(y * y + aZ * aZ)
    r2 = math.sqrt(r1 * r1 + aX * aX)

    L1, L2 = (LEFT1, LEFT2) if mirror else (RIGHT1, RIGHT2)
    if r2 > (L1 + L2):
        r2 = L1 + L2

    g1 = math.asin(aX / r2) * PHI
    g3 = math.acos((L1 * L1 + L2 * L2 - r2 * r2) / (2 * L1 * L2)) * PHI
    g2 = math.acos((L1 * L1 + r2 * r2 - L2 * L2) / (2 * L1 * r2)) * PHI

    a = [0.0] * 6
    a[0] = angle
    a[1] = math.atan2(aZ, y) * PHI
    a[2] = -(g1 + g2)
    a[3] = 180 - g3
    a[4] = a[2] + a[3]
    a[5] = a[1]
    a[2] = -(g1 + g2) - 10
    a[2] -= setA2

    return [v + 180 for v in a]


def hand_angles(s1, s3, s5, mirror):
    """Direct transcription of Hand(). Returns [s1,s3,s5] in old convention
    (0-360, 180=center)."""
    vals = [s1, s3, s5]
    return [360 - v for v in vals] if mirror else vals


def convert_angle_old(angle_deg, mirror):
    """Faithful to convertAngle(): map(angle,0,360,0,4095), then 4095-temp
    if mirror. LEGS ONLY -- the old firmware's second mirror stage, applied
    on top of Leg()'s own internal mirroring, for the left leg only
    (mirror=True). Missing this was the v1->v2 bug in KudaPose."""
    temp = angle_deg / 360.0 * 4095.0
    return (4095.0 - temp) if mirror else temp


def convert_angle2_old(angle_deg):
    """Faithful to convertAngle2(): plain map, no mirror stage, ever --
    used for arms/head in the old firmware."""
    return angle_deg / 360.0 * 4095.0


def old_tick_equiv_to_our_rad(tick_equiv):
    """tick 2048 = center = our 0 rad; 4095 ticks = 360 deg = 2*pi rad."""
    return (tick_equiv - 2048.0) / 4095.0 * 2 * math.pi


def _apply_extra_flip(pose):
    for name in EXTRA_FLIP:
        if name in pose:
            pose[name] = -pose[name]
    return pose


def build_leg_pose(right_leg_deg, left_leg_deg):
    """Legs only -- for movements (jalanDiTempat, putarKiri/Kanan,
    kepitingKanan/Kiri, ...) that in the old firmware never call Hand()/
    Body(), leaving hands/head at whatever they were previously set to.
    Publish this dict alone (not merged with a full 20-joint pose) so the
    bridge's per-joint last_known_positions leaves hands/head untouched,
    matching that behavior instead of snapping them to a guessed default."""
    pose = {}
    for name, deg in zip(RIGHT_LEG_NAMES, right_leg_deg):
        pose[name] = old_tick_equiv_to_our_rad(convert_angle_old(deg, mirror=False))
    for name, deg in zip(LEFT_LEG_NAMES, left_leg_deg):
        pose[name] = old_tick_equiv_to_our_rad(convert_angle_old(deg, mirror=True))
    return _apply_extra_flip(pose)


def build_arm_pose(hand_right_old, hand_left_old):
    """Arms only. hand_right_old maps to OUR Kiri arm, hand_left_old to OUR
    Kanan arm -- see module docstring on the arm ID side-swap."""
    pose = {}
    for name, deg in zip(OUR_KIRI_ARM_NAMES, hand_right_old):
        pose[name] = old_tick_equiv_to_our_rad(convert_angle2_old(deg))
    for name, deg in zip(OUR_KANAN_ARM_NAMES, hand_left_old):
        pose[name] = old_tick_equiv_to_our_rad(convert_angle2_old(deg))
    return _apply_extra_flip(pose)


def build_head_pose(leher=None, kepala=None):
    """Head/neck only, old Body()'s two arguments in the old 0-360 degree
    convention. A None argument omits that joint from the returned dict
    entirely (leaves it untouched), matching a function that never calls
    Body() at all."""
    pose = {}
    if leher is not None:
        pose['Leher Putar'] = old_tick_equiv_to_our_rad(convert_angle2_old(leher))
    if kepala is not None:
        pose['Kepala Putar'] = old_tick_equiv_to_our_rad(convert_angle2_old(kepala))
    return _apply_extra_flip(pose)


def build_leg_arm_pose(right_leg_deg, left_leg_deg, hand_right_old, hand_left_old):
    """Legs + arms, head omitted -- for movements (jalan()) that call
    Hand() every step but never Body(), leaving head untouched."""
    pose = {}
    pose.update(build_leg_pose(right_leg_deg, left_leg_deg))
    pose.update(build_arm_pose(hand_right_old, hand_left_old))
    return pose


def build_pose(right_leg_deg, left_leg_deg, hand_right_old, hand_left_old,
               leher=None, kepala=None):
    """Full 20-joint pose (legs + arms + head, head defaulting to our
    neutral 0 rad rather than being omitted) -- for single-shot static
    poses like kuda() that want every joint explicitly commanded. For a
    movement that only touches some joints, prefer the build_*_pose()
    functions above and publish just that subset.

    right_leg_deg / left_leg_deg: leg_ik() outputs (6-element lists).
    hand_right_old / hand_left_old: hand_angles() outputs (3-element lists).
    """
    pose = {}
    pose.update(build_leg_pose(right_leg_deg, left_leg_deg))
    pose.update(build_arm_pose(hand_right_old, hand_left_old))
    pose['Leher Putar'] = 0.0
    pose['Kepala Putar'] = 0.0
    pose.update(build_head_pose(leher, kepala))
    return pose


# Fixed, repo-root path -- deliberately OUTSIDE src/, so colcon build never
# touches it and editing it takes effect on the very next run. Originally
# KudaPose-only; centralized here since the per-joint sign correction it
# holds is a property of the joints themselves, not of any one movement.
FLIP_CONFIG_PATH = '/home/ariqmau/gladitos/Simulasi-Gazebo-Darnet/kuda_flip_config.json'


def load_flip_config():
    try:
        with open(FLIP_CONFIG_PATH) as f:
            raw = json.load(f)
    except Exception as e:
        print(f"(no flip config at {FLIP_CONFIG_PATH} or failed to read it: {e} -- using no flips)")
        return {}
    return {k: v for k, v in raw.items() if not k.startswith('_')}


def apply_flip_config(pose, verbose=True):
    """Applies the same per-run, no-rebuild-needed sign flips KudaPose.py
    uses (see FLIP_CONFIG_PATH) to any pose dict, in place, and returns it."""
    flips = load_flip_config()
    if verbose:
        print(f"\nApplying flip config from {FLIP_CONFIG_PATH}")
    for name in pose:
        sign = flips.get(name, 1)
        if sign not in (1, -1):
            print(f"  WARNING: {name} has invalid flip value {sign!r} in config, treating as 1")
            sign = 1
        before = pose[name]
        pose[name] = before * sign
        if verbose:
            marker = "  <-- FLIPPED" if sign == -1 else ""
            print(f"  {name:20s} base={before:+.4f}  ->  final={pose[name]:+.4f}{marker}")
    return pose
