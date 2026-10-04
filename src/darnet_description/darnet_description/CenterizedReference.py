#!/usr/bin/env python3
"""Preview or explicitly execute a SMALL raw-tick Centerized reference correction.

Default is offline preview (no SDK import, serial port, or files). --inspect is
read-only. --execute requires support/exclusive-port acknowledgements and an
interactive confirmation. Never writes EEPROM, torque enable/limit, or offsets.
NOT a stand-up controller, collision safety certificate or emergency stop.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

try:
    from .CaptureEncoderReference import capture_sample, read_value, infer_mode, snapshot_configuration
except ImportError:
    from CaptureEncoderReference import capture_sample, read_value, infer_mode, snapshot_configuration


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


def preflight(rows):
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
        if max(abs(value-TARGETS[servo_id]) for value in values[:2]) > MAX_START_DELTA:
            errors.append(prefix + 'too far from reference; this is NOT a stand-up tool')
        if abs(row['goal_ticks']-row['present_ticks']) > MAX_INITIAL_GOAL_ERROR:
            errors.append(prefix + 'present position too far from existing goal')
        if row['temperature_c'] >= min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5):
            errors.append(prefix + 'temperature guard exceeded')
        if not row['voltage_min_raw'] <= row['voltage_raw'] <= row['voltage_max_raw']:
            errors.append(prefix + 'voltage outside configured device limits')
    return errors


def next_frame(current):
    return {i: v + max(-STEP_TICKS, min(STEP_TICKS, TARGETS[i]-v)) for i, v in current.items()}


def checked_write(packet, port, servo_id, address, value):
    if address not in (30, 32):
        raise ValueError('Only Goal Position and Moving Speed RAM writes permitted')
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError('Integer register value required')
    if address == 30 and not 0 <= value <= 4095:
        raise ValueError('Goal outside single-turn tick range')
    if address == 32 and value != MOVING_SPEED_CAP:
        raise ValueError('Only the fixed nonzero speed cap is permitted')
    result, error = packet.write2ByteTxRx(port, servo_id, address, value)
    if result != 0 or error:
        raise RuntimeError(f'ID {servo_id}: write failed: {packet.getTxRxResult(result)} / {packet.getRxPacketError(error)}')
    actual = read_value(packet, port, servo_id, address, 2)
    if actual != value:
        raise RuntimeError(f'ID {servo_id}: register {address} readback mismatch')


def read_all(packet, port):
    return {i: capture_sample(packet, port, i, extended=True) for i in TARGETS}


def check_runtime(packet, port, baseline, goals):
    rows = {}
    fields = (('cw_limit', 6, 2), ('ccw_limit', 8, 2), ('resolution_divider', 22, 1),
              ('torque_enable', 24, 1), ('goal_ticks', 30, 2), ('moving_speed_raw', 32, 2),
              ('torque_limit', 34, 2), ('present_ticks', 36, 2),
              ('voltage_raw', 42, 1), ('temperature_c', 43, 1),
              ('registered_instruction', 44, 1), ('moving', 46, 1))
    for i in TARGETS:
        row = dict(baseline[i])
        for name, address, size in fields:
            row[name] = read_value(packet, port, i, address, size)
        if row['model_number'] == 310:
            row['torque_control_mode'] = read_value(packet, port, i, 70, 1)
        if infer_mode(row) != 'joint':
            raise RuntimeError(f'ID {i}: operating mode changed')
        if any(row[k] != baseline[i][k] for k in ('cw_limit', 'ccw_limit', 'resolution_divider')):
            raise RuntimeError(f'ID {i}: configuration changed')
        if row['torque_enable'] != 1 or row['torque_limit'] <= 0 or row['registered_instruction']:
            raise RuntimeError(f'ID {i}: torque/shutdown/registered-instruction guard')
        if row['moving_speed_raw'] != MOVING_SPEED_CAP or row['goal_ticks'] != goals[i]:
            raise RuntimeError(f'ID {i}: unexpected speed/goal; possible competing writer')
        if abs(row['present_ticks']-goals[i]) > MAX_TRACKING_ERROR:
            raise RuntimeError(f'ID {i}: tracking error; no more trajectory writes')
        if row['temperature_c'] >= min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5):
            raise RuntimeError(f'ID {i}: temperature guard exceeded')
        if not row['voltage_min_raw'] <= row['voltage_raw'] <= row['voltage_max_raw']:
            raise RuntimeError(f'ID {i}: voltage guard exceeded')
        rows[i] = row
    return rows


def execute_small_correction(packet, port, baseline, event, *, sleep=time.sleep, clock=time.monotonic):
    errors = preflight(baseline)
    if errors:
        raise RuntimeError('; '.join(errors))
    goals = {i: baseline[i]['goal_ticks'] for i in TARGETS}
    if goals == TARGETS:
        return {'status': 'ALREADY_AT_REFERENCE_GOALS_NO_WRITES', 'positions': baseline,
                'visual_pose_confirmation_required': True}
    # Set/verify ALL nonzero speed caps before any new goal. Do not restore old
    # speeds automatically: old value 0 means uncapped speed, not stop.
    for i in TARGETS:
        event('write_attempt', id=i, address=32, value=MOVING_SPEED_CAP)
        checked_write(packet, port, i, 32, MOVING_SPEED_CAP)
    event('speed_caps_set', value=MOVING_SPEED_CAP, restored_on_exit=False)
    frame_index = 0
    while goals != TARGETS:
        start = clock()
        rows = check_runtime(packet, port, baseline, goals)
        event('feedback_before_frame', frame=frame_index, positions=rows)
        planned = next_frame(goals)
        for i in TARGETS:
            if planned[i] != goals[i]:
                event('write_attempt', frame=frame_index, id=i, address=30, value=planned[i])
                checked_write(packet, port, i, 30, planned[i])
                goals[i] = planned[i]
                event('goal_written', frame=frame_index, id=i, goal=goals[i])
        frame_index += 1
        sleep(max(0., FRAME_PERIOD-(clock()-start)))
    # Require repeated settled feedback; goal write success != pose success.
    deadline = clock()+10.
    settled = 0
    while clock() < deadline:
        rows = check_runtime(packet, port, baseline, goals)
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
    parser.add_argument('--robot-supported', action='store_true')
    parser.add_argument('--exclusive-port-confirmed', action='store_true', help='Operator confirms bridge/Wizard/other writers stopped')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--output', type=Path, help='NEW log directory; existing directories are refused')
    options = parser.parse_args(args)
    if options.baud <= 0:
        parser.error('Positive baud required')
    if options.execute and not (options.robot_supported and options.exclusive_port_confirmed):
        parser.error('--execute requires --robot-supported and --exclusive-port-confirmed')
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
                 'torque_enable_writes': False, 'eeprom_writes': False,
                 'shutdown': 'no automatic torque/speed/goal restoration; last goals can still move',
                 'completed': False})
    port = None
    exit_code = 0
    with (destination/'events.jsonl').open('x', encoding='utf-8') as log:
        def event(kind, **values):
            log.write(json.dumps({'utc': datetime.now(timezone.utc).isoformat(),
                                 'event': kind, **values}, allow_nan=False)+'\n')
            log.flush()
        try:
            from dynamixel_sdk import PortHandler, PacketHandler
            port = PortHandler(options.port); packet = PacketHandler(1.)
            if not port.openPort() or not port.setBaudRate(options.baud):
                raise RuntimeError('Cannot open/configure U2D2 port')
            baseline = read_all(packet, port)
            show(baseline); event('initial_read_only_snapshot', positions=baseline)
            errors = preflight(baseline); meta['preflight_errors'] = errors
            if options.inspect:
                meta['status'] = 'READ_ONLY_INSPECTION_COMPLETE'
                print('Read-only inspection complete. Execute blockers:', errors)
            else:
                if errors:
                    raise RuntimeError('Preflight refused movement: '+'; '.join(errors))
                print('Support robot, verify clear path, close other writers, keep power cutoff ready.')
                print('Only RAM Goal Position and Moving Speed will change. Ctrl+C is NOT torque-off/emergency-stop.')
                if input('Type MOVE CENTERIZED to proceed: ').strip() != 'MOVE CENTERIZED':
                    raise RuntimeError('Operator did not confirm; no writes')
                # Refresh all evidence AFTER the potentially long human prompt.
                fresh = read_all(packet, port)
                if any(abs(fresh[i]['present_ticks']-baseline[i]['present_ticks']) > 8 for i in TARGETS):
                    raise RuntimeError('Robot moved during confirmation; restart inspection')
                event('fresh_preflight_snapshot', positions=fresh)
                result = execute_small_correction(packet, port, fresh, event)
                meta.update(result)
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
            print('Log folder:', destination.resolve())
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
