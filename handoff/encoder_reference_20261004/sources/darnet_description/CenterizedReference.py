#!/usr/bin/env python3
"""Preview or explicitly execute a raw-tick Centerized reference correction.

Default is offline preview (no SDK import, serial port, or files). --inspect is
read-only. --execute requires support/exclusive-port acknowledgements and an
interactive confirmation. --direct instead prepares torque, permits larger
corrections within configured register limits and omits interactive prompts.
Never writes EEPROM, torque limit or offsets. Direct mode may write torque
enable and hold goals. NOT a collision safety certificate or emergency stop.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time

try:
    from .CaptureEncoderReference import capture_sample, read_value, read_block, decode_register, infer_mode, snapshot_configuration
except ImportError:
    from CaptureEncoderReference import capture_sample, read_value, read_block, decode_register, infer_mode, snapshot_configuration


JOINT_NAMES = (
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
    'Tangan Kiri', 'Tangan Kanan', 'Paha Kanan Putar', 'Paha Kiri Putar',
    'Paha Atas Kanan', 'Paha Atas Kiri', 'Paha Bawah Kanan', 'Paha Bawah Kiri',
    'Lutut Kanan', 'Lutut Kiri', 'Kaki Kanan Atas', 'Kaki Kiri Atas',
    'Kaki Kanan Bawah', 'Kaki Kiri Bawah', 'Leher Putar', 'Kepala Putar',
)
TARGETS = {i: (1946 if i == 8 else 2081 if i == 12 else 2048) for i in range(1, 21)}
EXPECTED_MODELS = {i: (310 if 7 <= i <= 18 else 29) for i in TARGETS}
# Conservative ENGINEERING guardrails for near-reference corrections only.
MAX_START_DELTA = 128   # about 11.25 degrees; not a measured safe ROM
MAX_TRACKING_ERROR = 24
MAX_INITIAL_GOAL_ERROR = 16
STEP_TICKS = 4
FRAME_PERIOD = .2
MOVING_SPEED_CAP = 20  # nonzero RAM speed setting, not a calibrated velocity
MAX_TEMPERATURE_C = 55
FINAL_TOLERANCE_TICKS = 8


def preflight(rows, *, max_start_delta=MAX_START_DELTA):
    """All IDs must pass before ANY write; never silently omit a failed joint."""
    errors = []
    if set(rows) != set(TARGETS):
        return ['Require all 20 unique IDs']
    for servo_id, row in rows.items():
        prefix = f'ID {servo_id:02d}: '
        if row.get('model_number') != EXPECTED_MODELS[servo_id]:
            errors.append(prefix + 'model differs from reference mapping')
        if row.get('device_id') != servo_id:
            errors.append(prefix + 'device ID mismatch')
        if infer_mode(row) != 'joint':
            errors.append(prefix + 'requires single-turn JOINT mode, not wheel/multiturn/torque mode')
        if row.get('resolution_divider') != 1 or row.get('multiturn_offset_raw') != 0:
            errors.append(prefix + 'divider/offset is nonstandard; refusing automatic interpretation')
        if row.get('status_return_level') != 2:
            errors.append(prefix + 'requires status-return-level 2 for acknowledged writes')
        if row.get('torque_enable') != 1 or row.get('torque_limit', 0) <= 0:
            errors.append(prefix + 'torque must already be ON with nonzero limit; this tool never enables it')
        if row.get('registered_instruction') != 0:
            errors.append(prefix + 'pending registered instruction')
        if row.get('moving') != 0 or row.get('present_speed_raw', 0) % 1024 > 5:
            errors.append(prefix + 'servo is not stationary')
        lo, hi = row['cw_limit'], row['ccw_limit']
        values = (row['present_ticks'], row['goal_ticks'], TARGETS[servo_id])
        if not 0 <= lo < hi <= 4095 or any(not lo <= value <= hi for value in values):
            errors.append(prefix + 'present/goal/reference outside joint register limits')
        if max_start_delta is not None and max(abs(value-TARGETS[servo_id]) for value in values[:2]) > max_start_delta:
            errors.append(prefix + 'too far from reference; this is NOT a stand-up tool')
        if abs(row['goal_ticks']-row['present_ticks']) > MAX_INITIAL_GOAL_ERROR:
            errors.append(prefix + 'present position too far from existing goal')
        if row['temperature_c'] >= min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5):
            errors.append(prefix + 'temperature guard exceeded')
        if not row['voltage_min_raw'] <= row['voltage_raw'] <= row['voltage_max_raw']:
            errors.append(prefix + 'voltage outside configured device limits')
    return errors


def target_limit_errors(rows):
    """Before direct torque preparation, targets must fit every joint's limits."""
    if set(rows) != set(TARGETS):
        return ['Require all 20 unique IDs']
    return [f'ID {i:02d}: reference outside configured joint limits'
            for i, target in TARGETS.items()
            if not 0 <= rows[i]['cw_limit'] <= target <= rows[i]['ccw_limit'] <= 4095
            or rows[i]['cw_limit'] == rows[i]['ccw_limit']]


def next_frame(current):
    return {i: v + max(-STEP_TICKS, min(STEP_TICKS, TARGETS[i]-v)) for i, v in current.items()}


def read_all(packet, port):
    return {i: capture_sample(packet, port, i, extended=True) for i in TARGETS}


def sync_write(packet, port, address, values, event, *, frame=None):
    """Broadcast RAM writes; transmit success is NOT servo acceptance.

    Caller MUST check every ID's register readback/feedback before advancing.
    No retries on transmission failure; partial application can remain.
    """
    if (address not in (30, 32) or not values or not set(values) <= set(TARGETS) or
            any(type(i) is not int for i in values)):
        raise ValueError('Only supported IDs and goal/speed RAM Sync Write allowed')
    if address == 32 and set(values) != set(TARGETS):
        raise ValueError('Speed caps must cover all IDs before new goals')
    for value in values.values():
        if type(value) is not int or not (0 <= value <= 4095 if address == 30 else value == MOVING_SPEED_CAP):
            raise ValueError('Invalid Sync Write register value')
    params = [byte for i, value in sorted(values.items()) for byte in (i, value & 255, value >> 8)]
    event('sync_write_attempt', frame=frame, address=address, values=values)
    result = packet.syncWriteTxOnly(port, address, 2, params, len(params))
    if result != 0:
        raise RuntimeError(f'SYNC WRITE address={address} frame={frame}: {packet.getTxRxResult(result)}; '
                           'application may be partial/unknown; NO automatic write retry')
    event('sync_write_transmitted', frame=frame, address=address, values=values,
          servo_acceptance_verified=False)


def check_runtime(packet, port, baseline, goals, event=None):
    rows = {}
    fields = (('cw_limit', 6, 2), ('ccw_limit', 8, 2), ('resolution_divider', 22, 1),
              ('torque_enable', 24, 1), ('goal_ticks', 30, 2), ('moving_speed_raw', 32, 2),
              ('torque_limit', 34, 2), ('present_ticks', 36, 2),
              ('voltage_raw', 42, 1), ('temperature_c', 43, 1),
              ('registered_instruction', 44, 1), ('moving', 46, 1))
    for i in TARGETS:
        row = dict(baseline[i])
        data = read_block(packet, port, i, 0, 47)
        row['model_number'] = decode_register(data, 0, 0, 2)
        row['device_id'] = decode_register(data, 0, 3, 1)
        for name, address, size in fields:
            row[name] = decode_register(data, 0, address, size)
        if row['model_number'] != baseline[i]['model_number'] or row['device_id'] != i:
            raise RuntimeError(f'ID {i}: model/ID changed; no further model-specific reads or writes')
        if row['model_number'] == 310:
            row['torque_control_mode'] = read_value(packet, port, i, 70, 1)
        if event:
            event('runtime_joint_feedback', id=i, position=row)
        if infer_mode(row) != 'joint':
            raise RuntimeError(f'ID {i}: operating mode changed')
        if any(row[k] != baseline[i][k] for k in ('cw_limit', 'ccw_limit', 'resolution_divider')):
            raise RuntimeError(f'ID {i}: configuration changed')
        if row['torque_enable'] != 1 or row['torque_limit'] <= 0 or row['registered_instruction']:
            raise RuntimeError(f'ID {i}: torque/shutdown/registered-instruction guard')
        if row['moving_speed_raw'] != MOVING_SPEED_CAP or row['goal_ticks'] != goals[i]:
            raise RuntimeError(f"ID {i}: speed/goal readback mismatch: expected_speed={MOVING_SPEED_CAP} "
                               f"observed_speed={row['moving_speed_raw']} expected_goal={goals[i]} "
                               f"observed_goal={row['goal_ticks']}; cause not identified")
        if abs(row['present_ticks']-goals[i]) > MAX_TRACKING_ERROR:
            raise RuntimeError(f'ID {i}: tracking error; no more trajectory writes')
        if row['temperature_c'] >= min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5):
            raise RuntimeError(f"ID {i}: temperature guard exceeded: observed={row['temperature_c']} C "
                               f"threshold={min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5)} C")
        if not row['voltage_min_raw'] <= row['voltage_raw'] <= row['voltage_max_raw']:
            raise RuntimeError(f'ID {i}: voltage guard exceeded')
        rows[i] = row
    return rows


def execute_small_correction(packet, port, baseline, event, *, sleep=time.sleep, clock=time.monotonic,
                             max_start_delta=MAX_START_DELTA):
    errors = preflight(baseline, max_start_delta=max_start_delta)
    if errors:
        raise RuntimeError('; '.join(errors))
    goals = {i: baseline[i]['goal_ticks'] for i in TARGETS}
    if goals == TARGETS:
        return {'status': 'ALREADY_AT_REFERENCE_GOALS_NO_WRITES', 'positions': baseline,
                'visual_pose_confirmation_required': True}
    # Set/verify ALL nonzero speed caps before any new goal. Do not restore old
    # speeds automatically: old value 0 means uncapped speed, not stop.
    sync_write(packet, port, 32, {i: MOVING_SPEED_CAP for i in TARGETS}, event)
    frame_index = 0
    pending = None
    def verify_pending():
        nonlocal pending
        if pending is not None:
            frame, values = pending
            event('sync_write_verified', frame=frame, address=30, values=values)
            for i, value in values.items():
                event('goal_written', frame=frame, id=i, goal=value, register_readback_verified=True)
            pending = None
    while goals != TARGETS:
        start = clock()
        rows = check_runtime(packet, port, baseline, goals, event)
        verify_pending()
        if frame_index == 0:
            event('speed_caps_set', value=MOVING_SPEED_CAP, register_readback_verified=True, restored_on_exit=False)
        event('feedback_before_frame', frame=frame_index, positions=rows)
        planned = next_frame(goals)
        changed = {i: value for i, value in planned.items() if value != goals[i]}
        sync_write(packet, port, 30, changed, event, frame=frame_index)
        pending = (frame_index, changed)
        goals = planned
        frame_index += 1
        elapsed = clock()-start
        event('frame_timing', frame=frame_index-1, elapsed_s=elapsed,
              nominal_period_s=FRAME_PERIOD, overrun=elapsed > FRAME_PERIOD)
        sleep(max(0., FRAME_PERIOD-elapsed))
    # Require repeated settled feedback; goal write success != pose success.
    deadline = clock()+10.
    settled = 0
    while clock() < deadline:
        rows = check_runtime(packet, port, baseline, goals, event)
        verify_pending()
        event('settling_feedback', positions=rows)
        near = all(abs(rows[i]['present_ticks']-TARGETS[i]) <= FINAL_TOLERANCE_TICKS
                   and rows[i]['moving'] == 0 for i in TARGETS)
        settled = settled+1 if near else 0
        if settled >= 3:
            return {'status': 'REFERENCE_GOALS_REACHED_FEEDBACK_NEAR_TARGET',
                    'positions': rows, 'visual_pose_confirmation_required': True}
        sleep(FRAME_PERIOD)
    raise RuntimeError('Timeout settling; no automatic retry or torque shutdown')


def show(rows=None):
    print('USER-REPORTED RAW GOALS; no demo offsets or ROS bridge conversion.')
    for i, target in TARGETS.items():
        extra = '' if rows is None else f" present={rows[i]['present_ticks']} old_goal={rows[i]['goal_ticks']} torque={rows[i]['torque_enable']} mode={infer_mode(rows[i])}"
        print(f'ID {i:02d} {JOINT_NAMES[i-1]} -> {target}{extra}')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--inspect', action='store_true', help='Read-only hardware inspection, no writes')
    modes.add_argument('--execute', action='store_true', help='Explicitly request guarded near-reference motion')
    modes.add_argument('--direct', action='store_true',
                       help='Prepare torque and ramp to raw targets; allow larger correction and skip interactive prompts')
    parser.add_argument('--robot-supported', action='store_true')
    parser.add_argument('--exclusive-port-confirmed', action='store_true', help='Operator confirms bridge/Wizard/other writers stopped')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--hold-settle-timeout', type=float, default=2.0,
                        help='Direct-mode settling seconds per newly activated joint (0.1..10, default 2)')
    parser.add_argument('--output', type=Path, help='NEW log directory; existing directories are refused')
    options = parser.parse_args(args)
    if options.direct:
        options.execute = True
    if options.baud <= 0:
        parser.error('Positive baud required')
    if not .1 <= options.hold_settle_timeout <= 10:
        parser.error('--hold-settle-timeout must be finite and within 0.1..10 seconds')
    if options.execute and not (options.robot_supported and options.exclusive_port_confirmed):
        parser.error('--execute/--direct require --robot-supported and --exclusive-port-confirmed')
    if not options.execute and not options.inspect:
        show()
        print('OFFLINE PREVIEW ONLY. No serial port or files opened.')
        return 0
    destination = options.output or Path('centerized_reference_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    destination.mkdir(parents=True, exist_ok=False)
    meta = snapshot_configuration(destination)
    meta.update({'started_utc': datetime.now(timezone.utc).isoformat(), 'targets': TARGETS,
                 'reference_status': 'USER_REPORTED_NOT_AUTOMATIC_CALIBRATION',
                 'port': options.port, 'baud': options.baud, 'protocol': 1.,
                 'operation': 'execute' if options.execute else 'read_only_inspect',
                 'arguments': {**vars(options), 'output': str(destination)},
                 'source_sha256_this_script': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 'torque_enable_writes': options.direct, 'eeprom_writes': False,
                 'trajectory_write_strategy': 'Protocol 1.0 RAM Sync Write; all-ID register/encoder verification before next frame',
                 'read_strategy': 'contiguous blocks; MX-64 torque-mode register separately; not atomic snapshots',
                 'max_start_delta_ticks': None if options.direct else MAX_START_DELTA,
                 'interactive_confirmation': options.execute and not options.direct,
                 'torque_on_attempted_ids': [], 'torque_on_verified_ids': [],
                 'hold_goal_activation_possible_ids': [], 'torque_activation_observations': {},
                 'shutdown': 'no automatic torque/speed/goal restoration; last goals can still move',
                 'completed': False})
    port = None
    exit_code = 0
    latency_path = Path('/sys/bus/usb-serial/devices') / Path(options.port).resolve().name / 'latency_timer'
    try:
        meta['usb_latency_ms'] = int(latency_path.read_text().strip())
    except (OSError, ValueError):
        meta['usb_latency_ms'] = None
    if meta['usb_latency_ms'] is not None:
        print(f"USB latency: {meta['usb_latency_ms']} ms (read-only check; not changed by this tool).")
        if meta['usb_latency_ms'] > 1:
            print('Consider the documented 1 ms U2D2 latency setting before operation; frame timing is logged.')
    with (destination/'events.jsonl').open('x', encoding='utf-8') as log:
        def event(kind, **values):
            log.write(json.dumps({'utc': datetime.now(timezone.utc).isoformat(),
                                 'event': kind, **values}, allow_nan=False)+'\n')
            log.flush()
            if kind == 'write_attempt' and values.get('address') == 24:
                meta['torque_on_attempted_ids'].append(values['id'])
            if kind == 'hold_goal_activation_possible':
                meta['hold_goal_activation_possible_ids'].append(values['id'])
            if kind == 'torque_on_verified':
                meta['torque_on_verified_ids'].append(values['id'])
                meta['torque_activation_observations'][str(values['id'])] = values['observation']
                print(f"ID {values['id']:02d}: torque ON verified ({values['observation'].replace('_', ' ')})")
            if kind == 'hold_settling_started':
                print(f"ID {values['id']:02d}: checking hold settling (timeout {values['timeout_s']:g} s)")
        try:
            from dynamixel_sdk import PortHandler, PacketHandler
            port = PortHandler(options.port); packet = PacketHandler(1.)
            packet.read_retry_observer = lambda details: event('read_retry', **details)
            if not port.openPort() or not port.setBaudRate(options.baud):
                raise RuntimeError('Cannot open/configure U2D2 port')
            baseline = read_all(packet, port)
            show(baseline); event('initial_read_only_snapshot', positions=baseline)
            if options.direct:
                try:
                    from . import PrepareTorqueHold as torque_hold
                except ImportError:
                    import PrepareTorqueHold as torque_hold
                shutil.copy2(Path(torque_hold.__file__), destination/'PrepareTorqueHold.py')
                meta['prepare_torque_source_sha256'] = hashlib.sha256(Path(torque_hold.__file__).read_bytes()).hexdigest()
                errors = torque_hold.preflight(baseline) + target_limit_errors(baseline)
            else:
                errors = preflight(baseline)
            meta['preflight_errors'] = errors
            if options.inspect:
                meta['status'] = 'READ_ONLY_INSPECTION_COMPLETE'
                print('Read-only inspection complete. Execute blockers:', errors)
            else:
                if errors:
                    raise RuntimeError('Preflight refused movement: '+'; '.join(errors))
                print('Support robot, verify clear path, close other writers, keep power cutoff ready.')
                if options.direct:
                    print('DIRECT MODE: no prompt; prepare torque then ramp to raw targets. Larger correction explicitly enabled.')
                    print('Existing torque limits preserved (possibly maximum). OFF-joint goals/speeds and torque WILL change.')
                    print('A hold-goal write may activate torque; each joint is checked before preparing the next.')
                    print('Ramp uses Sync Write plus all-ID readback. Transmit success alone is not pose success.')
                    print('Register limits do NOT prove a collision-free path. Ctrl+C is NOT torque-off/emergency-stop.')
                    prepared = torque_hold.prepare_hold(packet, port, baseline, event,
                                                        settle_timeout=options.hold_settle_timeout)
                    meta['torque_preparation'] = prepared
                    event('torque_preparation_complete', status=prepared['status'])
                    fresh = read_all(packet, port)
                else:
                    print('Only RAM Goal Position and Moving Speed will change. Ctrl+C is NOT torque-off/emergency-stop.')
                    if input('Type MOVE CENTERIZED to proceed: ').strip() != 'MOVE CENTERIZED':
                        raise RuntimeError('Operator did not confirm; no writes')
                    # Refresh all evidence AFTER the potentially long human prompt.
                    fresh = read_all(packet, port)
                    if any(abs(fresh[i]['present_ticks']-baseline[i]['present_ticks']) > 8 for i in TARGETS):
                        raise RuntimeError('Robot moved during confirmation; restart inspection')
                event('fresh_preflight_snapshot', positions=fresh)
                result = execute_small_correction(packet, port, fresh, event,
                    max_start_delta=None if options.direct else MAX_START_DELTA)
                meta.update(result)
                print('Result:', result['status'])
            meta['completed'] = True
        except KeyboardInterrupt:
            meta['status'] = 'INTERRUPTED_LAST_GOALS_REMAIN'; exit_code = 130
            print('Interrupted: stopped further writes, NOT an emergency stop. Support robot; last goals remain.')
        except Exception as exc:
            meta['status'] = 'FAILED_OR_REFUSED'; meta['error'] = str(exc); exit_code = 1
            print('Stopped:', exc)
            print('No automatic rollback/torque-off. Existing or last-written goals may still be active.')
        finally:
            if port is not None:
                port.closePort()
            meta['finished_utc'] = datetime.now(timezone.utc).isoformat()
            (destination/'reference_metadata.json').write_text(json.dumps(meta, indent=2, allow_nan=False)+'\n', encoding='utf-8')
            if options.direct:
                print('Hold-goal writes that may activate torque, attempted IDs:', meta['hold_goal_activation_possible_ids'])
                print('Explicit torque-on write attempted IDs:', meta['torque_on_attempted_ids'])
                print('New torque-on verified IDs:', meta['torque_on_verified_ids'])
                print('Torque states are not changed on exit; no automatic torque-off or rollback.')
            print('Log folder:', destination.resolve())
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
