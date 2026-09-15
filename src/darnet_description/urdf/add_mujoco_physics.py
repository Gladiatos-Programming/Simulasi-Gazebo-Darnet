#!/usr/bin/env python3
"""
Third stage on top of xacro_to_mujoco.py: turns the kinematics-only MJCF
(fixed base, gravity off -- used for CalibrateInversion.py's reference
images and the plain kinematic viewer) into a real physics model:
- base_link becomes a free-floating body (freejoint) instead of being fused
  into worldbody, using its original URDF <inertial> (mass 1.26kg) instead
  of leaving it massless.
- gravity back on.
- a <position> actuator per joint (20 total), kp scaled off each joint's
  existing actuatorfrcrange (already present per-joint from the URDF
  <limit effort=...>), ctrlrange/forcerange matching the joint's own range.

Re-run this after re-running xacro_to_mujoco.py (which regenerates
darnet_mujoco.xml from scratch and would otherwise wipe this).

Usage:
  python3 xacro_to_mujoco.py && python3 add_mujoco_physics.py
"""
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(HERE, "darnet_mujoco.xml")

# From darnet.xacro's base_link <inertial> -- real mass/inertia, not a guess.
BASE_INERTIAL_POS = "0.12558372774814963 -0.07386522357602478 0.43247306079321574"
BASE_INERTIAL_MASS = "1.2606121985073475"
# URDF gives ixx iyy izz ixy iyz ixz; MJCF fullinertia wants ixx iyy izz ixy ixz iyz.
BASE_INERTIAL_FULLINERTIA = "0.00507 0.006653 0.003975 -1e-06 1e-06 -0.00033"

KP_PER_UNIT_FRCRANGE = 0.6  # heuristic starting stiffness, tune from observed behavior

tree = ET.parse(MJCF_PATH)
root = tree.getroot()

option = root.find('option')
option.set('gravity', '0 0 -9.81')

worldbody = root.find('worldbody')

move_to_base = []
for child in list(worldbody):
    if child.tag == 'light':
        continue
    if child.tag == 'geom' and child.get('name') == 'ground':
        continue
    move_to_base.append(child)
    worldbody.remove(child)

base_body = ET.SubElement(worldbody, 'body')
base_body.set('name', 'base_link')
base_body.set('pos', '0 0 0')

freejoint = ET.SubElement(base_body, 'freejoint')
freejoint.set('name', 'root')

inertial = ET.SubElement(base_body, 'inertial')
inertial.set('pos', BASE_INERTIAL_POS)
inertial.set('mass', BASE_INERTIAL_MASS)
inertial.set('fullinertia', BASE_INERTIAL_FULLINERTIA)

for el in move_to_base:
    base_body.append(el)

actuator = ET.SubElement(root, 'actuator')
for joint in root.iter('joint'):
    name = joint.get('name')
    frcrange = joint.get('actuatorfrcrange', '-20 20')
    frc_max = abs(float(frcrange.split()[1]))
    kp = frc_max * KP_PER_UNIT_FRCRANGE
    jrange = joint.get('range', '-3.14159 3.14159')

    act = ET.SubElement(actuator, 'position')
    act.set('name', name + '_act')
    act.set('joint', name)
    act.set('kp', f'{kp:.3f}')
    act.set('ctrlrange', jrange)
    act.set('forcerange', frcrange)

tree.write(MJCF_PATH)
print(f"wrote physics-enabled {MJCF_PATH}")
