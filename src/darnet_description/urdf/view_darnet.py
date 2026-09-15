import os
import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(here, "darnet_mujoco.xml")

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

print("Opening viewer... drag=rotate, scroll=zoom, close window to quit")
mujoco.viewer.launch(model, data)
