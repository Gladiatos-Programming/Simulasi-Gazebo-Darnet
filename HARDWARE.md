# Darnet Hardware Reference

Compiled from direct testing (Dynamixel Wizard, protocol probing), the codebase,
and team confirmation. Marked per item: **Confirmed** (tested or explicitly
confirmed by the team), **Documented** (found in code/comments, not independently
verified), or **Unknown** (not yet documented anywhere — ask the team).

## Servo bus

**Confirmed** (Dynamixel Wizard + direct protocol testing):
- U2D2 USB-to-TTL adapter, `/dev/ttyUSB0`, 1,000,000 bps, Dynamixel **Protocol 1.0**
- 20 servos total:
  - **8x Dynamixel MX-28** — IDs 1-6, 19, 20 (both arms + neck/head)
  - **12x Dynamixel MX-64** — IDs 7-18 (both legs — higher torque for
    load-bearing/locomotion)

## Power

**Confirmed** (team, 2026-09-14):
- Battery: **LiPo 3S, 11.1V nominal, 2200mAh**
- **LTC3780** module (adjustable synchronous buck-boost DC-DC converter) —
  sits between the battery and the custom power distribution PCB, regulating
  the LiPo's sagging voltage (~9.9V empty to ~12.6V full) down/up to a
  stable rail before distribution.
- Custom power distribution PCB — receives the LTC3780's regulated output
  and distributes power onward (to servos, onboard computer, etc.).
  Schematic/rail voltages/current ratings: **Unknown**.

## Onboard computer

**Documented, with an unresolved inconsistency**:
- `ComsROS2DARPUTbyJetson.py`'s docstring names a **Jetson Orin Nano**,
  describing the pipeline `Motion Script -> PinnochioIK -> /joint_trajectory
  -> [this node] -> Serial -> OpenRB`.
- `Darnet/Vision/README.md` (the vision subsystem) instead says **"Jetson
  Nano"** when explaining the choice of `yolov7-tiny`.
- These are different boards (Jetson Nano vs. Jetson Orin Nano) — **which one
  is actually on the robot is Unknown**, needs team confirmation.

## Secondary microcontroller

**Documented**: an **OpenRB-150** appears as an alternate/earlier motor-control
path (`ComsROS2OpenRB.py`, `ComsROS2OpenRBDARPUT.py`) — per project history,
this was the original firmware-based approach before the team moved to
driving the U2D2 directly from ROS2 on the Jetson. Whether it's still
physically wired in as a Jetson<->servo intermediary or fully retired:
**Unknown**.

## Camera

**Unknown** brand/model. Code only shows a generic `/camera/image_raw`
subscriber running YOLO inference (`Camera_testing.py`), and a *simulated*
Gazebo camera plugin (640x360 @ 30fps, in `darnet.xacro`) — that resolution
is the simulation's, not necessarily the real camera's.

## IMU

**Unknown** brand/model for the current robot. `imu_reader.py` subscribes to
a ros2_control broadcaster topic (sim-oriented). The only concrete IMU
reference in any of the repos is an MPU6050 in the legacy Arduino-era
`Previous-KRI-Code/GyroTest/GyroTest.ino` — not confirmed as what's on
Darnet now.

## Physical dimensions (measured from CAD/URDF, this session)

- Height: ~587 mm (standing straight, zero pose)
- Footprint: ~251 x 160 mm (arms at sides)
- Total mass: ~3.66 kg (from the URDF's declared inertial properties — a CAD
  estimate, not a scale measurement; plausible, since the 20 servos alone are
  rated at ~2.1 kg combined)

## Still needed to make this complete

- Confirm actual Jetson SKU (Nano vs. Orin Nano)
- LTC3780 module's set output voltage / current rating, and the power PCB's
  rail voltages/current ratings
- Camera model
- IMU model
- OpenRB-150's current status (in use / retired)
