#!/usr/bin/env python3
"""
MuJoCo stand-in for ComsROS2U2D2.py -- same input topic, same 20 joint
names, but drives the MuJoCo model instead of real servos. Lets any pose or
gait script (KudaPose, DiamDiTempat, JalanDiTempat, JalanKRI, ...) run
completely unchanged against the simulation: just run this node instead of
ComsROS2U2D2, in place of the real hardware bridge.

No zero-offset, no inversion, no per-servo ticks: the MJCF's joint names
(darnet_mujoco.xml, generated from the same xacro/URDF) are the exact same
strings as ORDERED_JOINT_NAMES, so our joint radians go straight into the
matching joint's position actuator (<name> + "_act").

darnet_mujoco.xml now has real physics: gravity on, base_link is a free
body (not welded to world) with its real URDF mass/inertia, ground contact
via each link's collision geom, and a position actuator per joint (kp
scaled off that joint's existing actuatorfrcrange -- see
add_mujoco_physics.py). So this can actually show the robot standing,
stepping, and (if the gait is bad) tipping over -- not just joint angles
moving in isolation. Physics is integrated in real sub-steps (mj_step),
not teleported, so a bad gait command can visibly fail here the same way
it would on real hardware (losing balance), which the earlier
kinematic-only version couldn't show.

Re-generating darnet_mujoco.xml from the xacro (xacro_to_mujoco.py) wipes
these physics additions -- re-run add_mujoco_physics.py after.

Run instead of ComsROS2U2D2.py:
  ros2 run darnet_description SimBridge
Then in another terminal, run any pose/gait script exactly as on real
hardware:
  ros2 run darnet_description JalanKRI
"""
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from darnet_description.ComsROS2U2D2 import ORDERED_JOINT_NAMES

MODEL_PATH = '/home/ariqmau/gladitos/Simulasi-Gazebo-Darnet/src/darnet_description/urdf/darnet_mujoco.xml'


TICK_PERIOD_S = 1.0 / 60.0


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.substeps = max(1, round(TICK_PERIOD_S / self.model.opt.timestep))
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Actuator targets interpolate the same way the real bridge's
        # commanded position ramps toward a goal over time_from_start --
        # but here it's a *target for the position actuator*, not a
        # teleported qpos, so gravity/contact can still overpower it.
        self.start_vals = {name: 0.0 for name in ORDERED_JOINT_NAMES}
        self.goal_vals = dict(self.start_vals)
        self.t0 = self.data.time  # simulated time, not wall-clock -- see tick()
        self.duration_s = 0.02

        self.sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback, 10)
        self.timer = self.create_timer(TICK_PERIOD_S, self.tick)

        self.get_logger().info(
            f"Sim bridge ready, viewer open. Loaded {MODEL_PATH}. "
            f"Run a pose/gait script in another terminal now.")

    def listener_callback(self, msg):
        if not msg.points:
            return
        # Simulated time (data.time), NOT wall-clock: if rendering makes a
        # tick take longer than TICK_PERIOD_S, wall-clock keeps advancing
        # but physics only advances by a fixed substeps*timestep per tick,
        # so a wall-clock-timed ramp would race through its motion in far
        # less *simulated* time than duration_s -- an effectively much
        # faster, more violent move than the script asked for, easily
        # enough to knock a standing robot off balance. Simulated time
        # can't outrun how much physics has actually been stepped.
        now = self.data.time
        # Snapshot current interpolation target (not actual qpos, which can
        # now differ from it under load) as the new start, so back-to-back
        # messages chain smoothly instead of jumping.
        self.start_vals = dict(self.goal_vals)

        point = msg.points[0]
        for name, pos in zip(msg.joint_names, point.positions):
            if name in self.goal_vals:
                self.goal_vals[name] = pos

        self.duration_s = max(point.time_from_start.sec + point.time_from_start.nanosec / 1e9, 0.02)
        self.t0 = now

    def tick(self):
        if not self.viewer.is_running():
            self.get_logger().info("Viewer closed, shutting down.")
            if rclpy.ok():
                rclpy.shutdown()
            return

        frac = min(1.0, (self.data.time - self.t0) / self.duration_s)
        for name in ORDERED_JOINT_NAMES:
            start = self.start_vals[name]
            goal = self.goal_vals[name]
            target = start + frac * (goal - start)
            self.data.actuator(name + '_act').ctrl[0] = target

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

        import numpy as np
        if not hasattr(self, '_dbg_i'):
            self._dbg_i = 0
        self._dbg_i += 1
        if self._dbg_i % 15 == 0:
            w, x, y, z = self.data.joint('root').qpos[3:7]
            pitch = np.degrees(np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
            xfrc_nonzero = np.nonzero(np.any(self.data.xfrc_applied != 0, axis=1))[0]
            xfrc_info = [(self.model.body(i).name, self.data.xfrc_applied[i].tolist()) for i in xfrc_nonzero]
            ncon = self.data.ncon
            self.get_logger().info(
                f"[DBG] t={self.data.time:.2f} frac={frac:.2f} pitch={pitch:+.2f} "
                f"base_z={self.data.joint('root').qpos[2]:+.4f} ncon={ncon} xfrc={xfrc_info}")


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
