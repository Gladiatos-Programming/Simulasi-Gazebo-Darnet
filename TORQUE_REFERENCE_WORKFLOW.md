# Torque preparation -> Centerized reference -> encoder capture

## Finalized handoff and result interpretation (4 October 2026)

The selected data, source snapshots, change history and limitations are in
[the consolidated handoff](handoff/encoder_reference_20261004/README.md).
Finalized packaging is NOT finalized geometric calibration.

`REFERENCE_GOALS_REACHED_FEEDBACK_NEAR_TARGET` is the repeated near-target
feedback result. `ALREADY_AT_REFERENCE_GOALS_NO_WRITES` currently bypasses the
final repeated-settling check and must NOT be treated as equivalent. Torque-ON
verification is holding preparation, not attainment of the final reference.

After Centerized exits and the robot is safely supported/still, a diagnostic
capture may be taken even after settling timeout, without `--pose-confirmed`.
Do not run capture concurrently on the same port. Use `--pose-confirmed` only
after independent geometric pose confirmation; this flag does not convert a
failed Centerized run into a PASS. Always check error rows, read_failures,
retry records and per-ID sample counts, not completed=True alone.

This workflow is for the existing legacy MX-28/MX-64 IDs 1..20 on Protocol 1.0.
It does not establish geometric zero, safe ROM, balance, or a safe stand-up path.
Do not run a bridge, Wizard, or another serial writer at the same time.

## One-command direct mode (explicit operator opt-in)

Requested after the operator reported manually testing the joints. Normal
--execute still has the original 128-tick proximity guard and prompt. --direct
removes ONLY that proximity guard and interactive prompts, prepares torque for
OFF joints using per-joint hold-current-pose checks, then ramps to the targets.
It still requires the two acknowledgements and has all model/ID, joint limits,
feedback, voltage/temperature and transport checks. It does NOT make the motion
path collision-free or make a stand-up trajectory suitable for an unsupported
robot. A tested endpoint is not proof of safety along every simultaneous path.
Existing torque limits (possibly maximum) are preserved.

After closing other writers and mechanically supporting the robot, manually run:

```bash
source /opt/ros/humble/setup.bash
source /home/orin/Simulasi-Gazebo-Darnet/install/setup.bash
ros2 run darnet_description CenterizedReference \
  --direct --robot-supported --exclusive-port-confirmed
```

There is no confirmation text to type. Targets: ID 8 -> 1946, ID 12 -> 2081,
all other IDs -> 2048. Movement remains a feedback-checked 4-tick/frame ramp
with a nominal 0.2-second frame period (actual rate depends on communications),
not an instant jump. Intermediate values (for example 2138 on a joint starting
above 2048) are ramp steps, not changed final targets. Do NOT run separate
PrepareTorqueHold first for this mode.
If preparation or correction fails, do not proceed to pose-confirmed capture.
Partial activations and last goals remain; no automatic rollback/torque-off.

Only after success and actual visual/geometric pose confirmation:

```bash
ros2 run darnet_description CaptureEncoderReference --samples 5 --pose-confirmed
```

## New tool: PrepareTorqueHold

- Default is offline preview: no SDK import, serial port or output files.
- --inspect reads all IDs and reports blockers; no motor writes.
- --execute requires support/exclusive-port acknowledgements plus an exact
  interactive confirmation. These acknowledgements do NOT enforce a port lock.
- Reads all IDs, requires supported model/ID, stationary single-turn joint mode,
  nonzero existing torque limits, Status Return Level 2, healthy configured
  voltage/temperature and no pending registered instruction.
- Torque-OFF joints are handled one at a time: Moving Speed (default 20,
  nonzero), Goal Position from captured Present Position, immediate full
  feedback check, explicit Torque Enable=1 ONLY if still OFF, then verified
  feedback before touching the next joint. A goal write MAY activate torque:
  there is no assumption that goal staging is passive. A fresh snapshot after
  the prompt and repeated guards reject pose/config drift. There is no bulk
  torque-OFF reset and no new goal/speed write to an initially-ON joint.
- Already-ON goals/speeds are unchanged. Torque Limit, EEPROM, offsets and
  bridge/controller behavior are unchanged. Existing torque limits may be 1023:
  that is not a measured safe physical limit and is not silently reduced/raised.
- Activation is automated across all IDs but acknowledged/read back individually.
  ON observed immediately after this tool's successful hold-goal write is
  accepted only if goal, speed, configuration, health and pose also pass.
  After activation, transient goal-command processing is allowed a bounded
  settling window before the joint is declared ready (details below).
  Unexpected state changes before that goal write remain errors. This temporal
  observation is NOT a firmware diagnosis or proof of exclusive access.
  No unverified broadcast torque activation. A successful readback is evidence at
  that time only, not a guarantee that overload/shutdown cannot occur later.
- There is NO automatic rollback or torque-off. If a write acknowledgement times
  out, the write might have applied. Stop rather than retry it blindly. Logs
  distinguish explicit torque write attempts, hold-goal writes that may activate
  torque, and verified ON observations. Existing ON joints
  also remain energized. Ctrl+C is not an emergency stop.
- Mechanical support, load clearance and an accessible power cutoff are required.
  Enabling torque can cause motion under load even with a near-current goal.

Engineering guards: maximum pose drift 8 ticks, already-ON goal error 16 ticks,
temperature below min(55 C, configured limit minus 5 C), present-speed magnitude
at most 5 raw units, Moving flag 0. Default speed cap 20 is not a universal safe
velocity; select it only after assessing the mechanism. Reads are sequential,
not simultaneous snapshots. These checks cannot detect all mechanical hazards.

## Commands (manually run by operator only)

Build the new entry point, then source the workspace:

```bash
cd /home/orin/Simulasi-Gazebo-Darnet
source /opt/ros/humble/setup.bash
colcon build --packages-select darnet_description --symlink-install
source install/setup.bash
```

First preview / read-only inspect:

```bash
ros2 run darnet_description PrepareTorqueHold
ros2 run darnet_description PrepareTorqueHold --inspect
```

Only if the robot is mechanically supported and all preflight checks pass:

```bash
ros2 run darnet_description PrepareTorqueHold \
  --execute --robot-supported --exclusive-port-confirmed
```

Read the live values and warning, then type the requested confirmation:
`HOLD CURRENT POSE AND ENABLE TORQUE`.

This only holds the current pose. It does NOT move to Centerized reference.
Next inspect the near-reference correction:

```bash
ros2 run darnet_description CenterizedReference --inspect
```

CenterizedReference still requires all torque ON, valid status/configuration,
and present/existing goal within 128 ticks of its targets. It is NOT a stand-up
controller. If a joint is too far away, do not increase the guard to force it;
arrange a separately supervised positioning procedure before using this tool.
Do not manually reposition a resisting energized joint. Support/repositioning
and any torque-off procedure require deliberate operator handling.

Only after those guards pass and the physical path is verified:

```bash
ros2 run darnet_description CenterizedReference \
  --execute --robot-supported --exclusive-port-confirmed
```

Only after successful feedback and visual confirmation that the intended
geometric reference pose is achieved, run the requested READ-only capture:

```bash
ros2 run darnet_description CaptureEncoderReference --samples 5 --pose-confirmed
```

`--pose-confirmed` records the OPERATOR's assertion. Five samples do not prove
geometric zero or automatically apply calibration/offsets. Torque remains as-is.
Inspect every row for errors and the metadata's read_failures / read_retries.

## Communication change

### Bounded hold settling after activation

Immediately after this tool's own hold-goal write / explicit torque enable,
`Moving(46)=1` can indicate goal-command execution even while measured speed
is zero. Faster USB/block-read turnaround exposed an overly early stationary
check. Newly activated joints now get a default 2-second READ-only settling
window, polling every 50 ms and requiring three consecutive observations with
Moving=0 and speed magnitude <=5 raw units. Each joint must settle BEFORE
preparation of the next joint or Centerized ramp begins.

Only the stationary requirement is deferred within this window. Pose drift
still must stay within 8 ticks of the captured pose on every read; goal, speed
setting, torque ON, configuration, joint limits, health and pending-instruction
checks remain strict. Invalid movement/speed registers are errors, not reasons
to wait. USB/read/retry time counts against the monotonic deadline. A timeout,
failed read, changed state or drift stops further writes without retrying goals
or automatically switching torque off. Initial/fresh preflight, pre-write
snapshots, already-ON joints, and final all-ID checks still require stationarity.

These are engineering thresholds, not universal physical limits. Both tools
accept `--hold-settle-timeout SECONDS` (0.1..10, default 2; relevant to newly
activated joints, only --direct for CenterizedReference). Do not extend the
timeout to hide actual drift or persistent motion. Logs include
`hold_settling_started`, `hold_settling_feedback`, and `hold_settled`; failure
messages include the last Moving flag, speed, goal and position. A verified ON
and settled observation is recorded only after the full checks pass.

ROBOTIS documents the Moving flag as command execution state in its
[MX-28 control table](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/#moving-46).
The brief-transient diagnosis is an inference from the run, not proof that a
joint cannot move under load.

### Block reads and verified Sync Write ramp

Snapshots still check the model first using an address-0 read. Supported legacy
MX models then use one contiguous address-2..46 register block; MX-64 torque
mode at address 70 is read separately. This reduces an extended snapshot from
roughly 29/30 transactions to 2/3 per joint while preserving the same fields.
Unsupported models are rejected before reading legacy-model blocks.

Runtime reads use a contiguous address-0..46 block per joint plus address 70
only for previously validated MX-64 models (32 transactions across 20 joints,
instead of 252). Model and ID are checked before the model-specific read.
Block length and bytes are validated; failed/truncated data is not used as
positions. These are register blocks, NOT simultaneous/atomic samples across
registers or motors. Model/reserved bytes in blocks are not interpreted except
at the documented register offsets.

The Centerized ramp now sends one Protocol-1.0 two-byte Sync Write for all speed
caps, then verifies every ID before any new goal. Each subsequent frame sends
one Sync Write for changed goals. Broadcasts have no individual status ACKs:
`sync_write_transmitted` is NOT servo acceptance. A full register/health/encoder
readback verifies each prior frame before the next goal packet. Last-frame
readback and three near-target stationary checks are required for success.
`goal_written` events now denote verified register readback, not transmission
alone. Transmission failure is not retried; applied state may be partial.
Rejected goal/speed readback, failed reads, tracking/state/configuration/health
checks stop further frames. No write retries or silent skipping of IDs.

`runtime_joint_feedback` is logged before each joint's guard evaluation;
temperature errors include observed temperature and threshold. `frame_timing`
records actual processing time and whether nominal 0.2 seconds was exceeded.
This optimization does not certify the hardware link or automatically remove
existing guards. Torque preparation remains per-joint unicast write/readback,
because that stage can activate loaded joints.

### U2D2 USB latency (explicit host setting, no motor operation)

The tool reads/logs the connected port's `latency_timer` but never changes it.
ROBOTIS recommends 1 ms in its [U2D2 latency guidance](https://emanual.robotis.com/docs/en/parts/interface/u2d2/#usb-latency-setting).
After closing Wizard/bridge and confirming the intended U2D2 is ttyUSB0, the
operator can apply this temporary host setting before manually running motion:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

This does not write a motor register or move a joint. It may revert after USB
reconnection/reboot; recheck the script's `USB latency` output. No global udev
rules or SDK files are modified. SDK timeout margin remains unchanged: lowering
USB latency is not grounds to blindly tighten timeouts. Lower latency and fewer
transactions improve turnaround, not proof that packet loss is eliminated.

### Goal writes may activate torque

The old prepare-all-OFF-goals-then-enable-all sequence assumed that every joint
would stay OFF during goal staging. That assumption is no longer used. Each
OFF joint now completes its own preparation and feedback validation before the
next goal is written. This supports both synthetic/tested possibilities:
torque stays OFF until explicit enable, or ON is observed after the goal write.
No automatic OFF cycle was added: OFF releases load support and is not a
firmware reset or goal-clear operation.

ROBOTIS's [R+ Manager documentation](https://emanual.robotis.com/docs/en/software/rplus1/manager/#torque-enable)
describes activation on goal update in that workflow. It does not establish the
cause of the observed transition for every MX firmware or raw SDK call. The
implementation handles the observed state without asserting that causality.

Metadata fields:

- `torque_on_attempted_ids`: explicit address-24 writes only (legacy field).
- `hold_goal_activation_possible_ids`: recorded BEFORE each OFF-joint goal
  write; torque may be active even if its ACK/readback later fails.
- `torque_on_verified_ids`: initially-OFF joints with verified ON feedback.
- `torque_activation_observations`: `on_observed_after_hold_goal_write` or
  `on_after_explicit_torque_write`, not a claimed firmware activation cause.

`after_hold_goal_write` snapshots precede guard checks. A goal timeout, invalid
feedback or failure stops further preparation/centering without automatic write
retry, torque-off or rollback; previously handled joints can remain ON. Logs
retain both attempted operations and successfully verified observations.

### Snapshot comparison and bounded read retries

Preparation snapshot checks distinguish an OFF joint's unowned old goal from
the hold goal written by this tool. Before preload only, a goal change while
torque remains OFF is logged but is not itself a competing-writer diagnosis:
the tool will replace that old goal with its measured-pose goal. Configuration,
torque state, actual pose drift and health checks remain strict. After preload,
and for joints already ON, goal equality/readback checks remain strict. A valid
OFF-to-ON observation after this tool's own hold-goal write is handled as above;
other torque transitions remain checked against expected state.
Guard errors now include previous/observed values and do not attribute a cause
without evidence. Extra snapshots are logged BEFORE guard evaluation, including
the fresh pre-write batch, preload reads and torque activation feedback.
This is a correction to the software assumption, not proof of any particular
servo firmware behavior and not a guarantee that subsequent hardware checks pass.

Shared read_value now makes at most 3 attempts for SDK READ receive timeout
(-3001) or corrupt response (-3002), with 20 ms between attempts. Device errors,
port-busy or transmit failures abort immediately. Final errors include ID,
register address, size and attempt. Successful retries are recorded in capture
metadata or Centerized/torque event logs. No automatic WRITE retries were added.
The same retry policy applies to block reads.
This improves observability/tolerance, not a certified fix for the physical
source of intermittent timeouts. Exclusive access, power/cabling and timing
still require validation; hardware was not exercised by the assistant.

## Results to retain

- torque_hold_*/: events.jsonl, torque_hold_metadata.json and source snapshots.
- centerized_reference_*/: events.jsonl, reference_metadata.json and snapshots.
- encoder_capture_*/: encoder_reference_centerized.csv, capture_metadata.json
  and configuration/source snapshots.

No historical run data is rewritten. Run folders are always new. Store all
three stages together so the capture can be traced to a verified holding and
near-reference correction session. A failure during any stage invalidates an
automatic claim that the robot is at the requested reference.

Offline verification (never connects to hardware):

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_*reference.py'
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_encoder_capture.py'
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_torque_hold.py'
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_direct_center.py'
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_block_sync_transport.py'
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s src/darnet_description/test -p 'test_hold_settling.py'
```
