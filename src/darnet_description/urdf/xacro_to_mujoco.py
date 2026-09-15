import re
import os
import mujoco

pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
xacro_path = os.path.join(pkg_dir, "urdf/darnet.xacro")
out_path = os.path.join(pkg_dir, "urdf/darnet_mujoco.urdf")
mjcf_path = os.path.join(pkg_dir, "urdf/darnet_mujoco.xml")

with open(xacro_path) as f:
    content = f.read()

# drop ROS2/Gazebo-only includes (ros2_control, transmissions, sensor plugins) -
# not needed for viewing geometry in MuJoCo
content = re.sub(r'<xacro:include filename="\$\(find darnet_description\)/urdf/darnet\.trans" />\n', '', content)
content = re.sub(r'<xacro:include filename="\$\(find darnet_description\)/urdf/darnet\.gazebo" />\n', '', content)

mujoco_extension = (
    '<mujoco>\n'
    '  <compiler discardvisual="false"/>\n'
    '  <option gravity="0 0 0"/>\n'
    '</mujoco>\n\n'
)

content = content.replace(
    '<xacro:include filename="$(find darnet_description)/urdf/materials.xacro" />',
    mujoco_extension +
    '<material name="silver">\n  <color rgba="0.700 0.700 0.700 1.000"/>\n</material>'
)

content = content.replace("$(find darnet_description)", pkg_dir)
content = content.replace('filename="file://', 'filename="')

with open(out_path, "w") as f:
    f.write(content)

print("wrote", out_path)

# --- second pass: compile the URDF, then add presentation-only extras
# (skybox, ground plane, light) directly in MJCF, since MuJoCo's URDF
# <mujoco> extension only honors <compiler>/<option>, not <asset>/<worldbody>.
model = mujoco.MjModel.from_xml_path(out_path)
mujoco.mj_saveLastXML(mjcf_path, model)
with open(mjcf_path) as f:
    mjcf = f.read()

asset_block = (
    '  <asset>\n'
    '    <texture name="skybox" type="skybox" builtin="gradient" '
    'rgb1="0.45 0.55 0.65" rgb2="0.06 0.07 0.09" width="512" height="512"/>\n'
    '    <texture name="groundtex" type="2d" builtin="checker" '
    'rgb1="0.2 0.2 0.22" rgb2="0.3 0.3 0.32" width="300" height="300"/>\n'
    '    <material name="groundmat" texture="groundtex" texrepeat="6 6" reflectance="0.1"/>\n'
    '  </asset>\n'
)
mjcf = mjcf.replace("<worldbody>", asset_block + "  <worldbody>\n"
    '    <light directional="true" pos="0 0 3" dir="0 0 -1" diffuse="0.7 0.7 0.7"/>\n'
    '    <geom name="ground" type="plane" size="2 2 0.05" material="groundmat" group="1"/>\n',
    1)

with open(mjcf_path, "w") as f:
    f.write(mjcf)

print("wrote", mjcf_path)
