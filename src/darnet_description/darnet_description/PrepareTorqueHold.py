#!/usr/bin/env python3
"""Guarded torque preparation for legacy MX IDs 1..20; NOT a pose controller.

Default: offline preview. --inspect: READ only. --execute requires physical
support, exclusive-port acknowledgement and interactive confirmation.
For OFF servos only: set nonzero Moving Speed, write Goal to measured Present
Position, immediately check for torque activation, explicitly enable if still
OFF, and verify feedback before preparing the next joint. A goal write MAY
activate torque; do not assume staging is passive. ON goals/speeds unchanged. Preserve
existing torque limits; never writes EEPROM, offsets or Torque Limit.
Enabling torque can cause motion under load. Sequential reads are not atomic.
No automatic rollback/torque-off; partial activation can remain energized.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time

try:
    from .CaptureEncoderReference import capture_sample, infer_mode, read_value, snapshot_configuration
    from .CenterizedReference import EXPECTED_MODELS, JOINT_NAMES
except ImportError:
    from CaptureEncoderReference import capture_sample, infer_mode, read_value, snapshot_configuration
    from CenterizedReference import EXPECTED_MODELS, JOINT_NAMES


IDS = tuple(range(1, 21))
MAX_POSE_DRIFT_TICKS = 8
MAX_ON_GOAL_ERROR_TICKS = 16
MAX_TEMPERATURE_C = 55
DEFAULT_SPEED_CAP = 20  # Engineering setting, NOT a universally safe physical speed.
DEFAULT_SETTLE_TIMEOUT = 2.0  # Seconds per newly activated joint, not a safety certificate.
SETTLE_POLL_INTERVAL = .05
SETTLE_STABLE_SAMPLES = 3
CONFIG_FIELDS = ('model_number', 'device_id', 'cw_limit', 'ccw_limit',
                 'resolution_divider', 'multiturn_offset_raw', 'status_return_level',
                 'torque_control_mode', 'torque_limit', 'max_torque',
                 'temperature_limit_c', 'voltage_min_raw', 'voltage_max_raw',
                 'p_gain', 'i_gain', 'd_gain', 'baud_register', 'return_delay')


def row_errors(servo_id, row, *, require_stationary=True):
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(f'ID {servo_id:02d}: {message}')
    require(row['model_number'] == EXPECTED_MODELS[servo_id], 'unexpected model')
    require(row['device_id'] == servo_id, 'device ID mismatch')
    require(infer_mode(row) == 'joint', 'single-turn JOINT mode required')
    require(row['resolution_divider'] == 1 and row['multiturn_offset_raw'] == 0,
            'nonstandard resolution/offset')
    require(row['status_return_level'] == 2, 'Status Return Level 2 required; no EEPROM adjustment here')
    require(row['torque_enable'] in (0, 1), 'invalid torque state')
    require(0 < row['torque_limit'] <= row['max_torque'] <= 1023,
            'existing nonzero valid torque limit required; this tool does not increase it')
    require(row['registered_instruction'] == 0, 'pending registered instruction')
    require(row['moving'] in (0, 1) and 0 <= row['present_speed_raw'] <= 2047,
            'invalid movement flag/speed register')
    if require_stationary:
        require(row['moving'] == 0 and row['present_speed_raw'] % 1024 <= 5,
                f"servo not stationary: moving={row['moving']} speed_raw={row['present_speed_raw']}")
    lo, hi = row['cw_limit'], row['ccw_limit']
    require(0 <= lo < hi <= 4095 and lo <= row['present_ticks'] <= hi,
            'present position / joint limits invalid')
    require(row['temperature_c'] < min(MAX_TEMPERATURE_C, row['temperature_limit_c']-5),
            'temperature guard exceeded')
    require(row['voltage_min_raw'] <= row['voltage_raw'] <= row['voltage_max_raw'],
            'voltage outside configured limits')
    if row['torque_enable'] == 1:
        require(lo <= row['goal_ticks'] <= hi and
                abs(row['goal_ticks']-row['present_ticks']) <= MAX_ON_GOAL_ERROR_TICKS,
                'already-ON servo is not holding near its existing goal')
    return errors


def preflight(rows):
    if set(rows) != set(IDS):
        return ['All 20 unique IDs must be read before any write']
    return [error for i in IDS for error in row_errors(i, rows[i])]


def read_all(packet, port):
    return {i: capture_sample(packet, port, i, extended=True) for i in IDS}


def check_row(servo_id, row, expected, pose, *, check_goal=True, require_stationary=True):
    errors = row_errors(servo_id, row, require_stationary=require_stationary)
    if errors:
        raise RuntimeError('; '.join(errors))
    for field in (*CONFIG_FIELDS, 'torque_enable', 'goal_ticks', 'moving_speed_raw'):
        # Before OFF-joint preload, the previous goal is not a command this
        # tool owns. Its variation alone does not prove a competing writer.
        if field == 'goal_ticks' and not check_goal and expected['torque_enable'] == 0 and row['torque_enable'] == 0:
            continue
        if row.get(field) != expected.get(field):
            raise RuntimeError(f'ID {servo_id:02d}: {field} changed: '
                               f"previous={expected.get(field)!r}, observed={row.get(field)!r}; cause not identified")
    if abs(row['present_ticks']-pose) > MAX_POSE_DRIFT_TICKS:
        raise RuntimeError(f'ID {servo_id:02d}: pose drift exceeds {MAX_POSE_DRIFT_TICKS} ticks: '
                           f"captured={pose}, observed={row['present_ticks']}; stop and re-inspect")


def wait_for_hold_settle(packet, port, servo_id, initial, expected, pose, event, *,
                         timeout=DEFAULT_SETTLE_TIMEOUT, sleep=time.sleep, clock=time.monotonic):
    """READ-only bounded settling after our activation; all other guards strict.

    Moving(46) indicates goal-command processing, not necessarily nonzero speed.
    Stationarity must be observed repeatedly; elapsed USB/read time counts
    toward the deadline. No other joint is prepared until this returns.
    """
    if not .1 <= timeout <= 10:
        raise ValueError('Hold settle timeout must be finite and within 0.1..10 seconds')
    start = clock()
    deadline = start+timeout
    row = initial
    stable = 0
    event('hold_settling_started', id=servo_id, timeout_s=timeout,
          poll_interval_s=SETTLE_POLL_INTERVAL, required_stable_samples=SETTLE_STABLE_SAMPLES)
    while True:
        elapsed = clock()-start
        event('hold_settling_feedback', id=servo_id, elapsed_s=elapsed, position=row)
        check_row(servo_id, row, expected, pose, require_stationary=False)
        stationary = row['moving'] == 0 and row['present_speed_raw'] % 1024 <= 5
        stable = stable+1 if stationary else 0
        elapsed = clock()-start
        if elapsed < timeout and stable >= SETTLE_STABLE_SAMPLES:
            event('hold_settled', id=servo_id, elapsed_s=elapsed, stable_samples=stable, position=row)
            return row
        remaining = deadline-clock()
        if remaining <= 0:
            raise RuntimeError(f'ID {servo_id:02d}: hold settling timed out after {timeout:g} s: '
                               f"moving={row['moving']} speed_raw={row['present_speed_raw']} "
                               f"present={row['present_ticks']} goal={row['goal_ticks']}; no further joint activation")
        sleep(min(SETTLE_POLL_INTERVAL, remaining))
        row = capture_sample(packet, port, servo_id, extended=True)


def checked_write(packet, port, servo_id, address, value, event):
    if address == 24:
        valid = type(value) is int and value == 1
        size = 1
    elif address == 30:
        valid = type(value) is int and 0 <= value <= 4095
        size = 2
    elif address == 32:
        valid = type(value) is int and 1 <= value <= 1023
        size = 2
    else:
        raise ValueError('Only RAM Torque Enable=1, Goal Position and nonzero Moving Speed allowed')
    if not valid:
        raise ValueError('Invalid RAM write value')
    event('write_attempt', id=servo_id, address=address, value=value)
    method = packet.write1ByteTxRx if size == 1 else packet.write2ByteTxRx
    result, error = method(port, servo_id, address, value)
    # A timeout after WRITE is ambiguous: it might have applied. Never retry it.
    if result != 0 or error:
        raise RuntimeError(f'ID {servo_id:02d} WRITE address={address}: '
                           f'{packet.getTxRxResult(result)} / {packet.getRxPacketError(error)}; '
                           'write outcome may be unknown; NO automatic write retry')
    if read_value(packet, port, servo_id, address, size) != value:
        raise RuntimeError(f'ID {servo_id:02d} WRITE address={address}: readback mismatch')
    event('write_verified', id=servo_id, address=address, value=value)


def prepare_hold(packet, port, baseline, event, *, speed_cap=DEFAULT_SPEED_CAP,
                 settle_timeout=DEFAULT_SETTLE_TIMEOUT):
    if type(speed_cap) is not int or not 1 <= speed_cap <= 1023:
        raise ValueError('Nonzero speed cap 1..1023 required')
    if not .1 <= settle_timeout <= 10:
        raise ValueError('Hold settle timeout must be finite and within 0.1..10 seconds')
    errors = preflight(baseline)
    if errors:
        raise RuntimeError('; '.join(errors))
    expected = {i: dict(row) for i, row in baseline.items()}
    poses = {i: row['present_ticks'] for i, row in baseline.items()}
    off_ids = [i for i in IDS if baseline[i]['torque_enable'] == 0]
    fresh = read_all(packet, port)
    event('fresh_read_only_snapshot', positions=fresh)
    for i in IDS:
        check_row(i, fresh[i], expected[i], poses[i], check_goal=expected[i]['torque_enable'] == 1)
        if expected[i]['torque_enable'] == 0 and fresh[i]['goal_ticks'] != expected[i]['goal_ticks']:
            event('off_goal_changed_before_preload', id=i, previous=expected[i]['goal_ticks'],
                  observed=fresh[i]['goal_ticks'], present=fresh[i]['present_ticks'])
    event('all_ids_verified_before_any_write', positions=fresh)

    # A Goal Position write may itself activate torque. Handle each joint
    # completely before touching the next; never stage all goals as if passive.
    activation_observations = {}
    for i in off_ids:
        row = capture_sample(packet, port, i, extended=True)
        event('before_speed_preload', id=i, position=row)
        check_row(i, row, expected[i], poses[i], check_goal=False)
        checked_write(packet, port, i, 32, speed_cap, event)
        expected[i]['moving_speed_raw'] = speed_cap
        # Check again: never preload a goal if torque unexpectedly turned ON.
        row = capture_sample(packet, port, i, extended=True)
        event('before_goal_preload', id=i, position=row)
        check_row(i, row, expected[i], poses[i], check_goal=False)
        # Record BEFORE the write: a timeout/mismatch may leave torque active.
        event('hold_goal_activation_possible', id=i, goal=poses[i])
        checked_write(packet, port, i, 30, poses[i], event)
        expected[i]['goal_ticks'] = poses[i]
        row = capture_sample(packet, port, i, extended=True)
        event('after_hold_goal_write', id=i, position=row)
        if row['torque_enable'] not in (0, 1):
            raise RuntimeError(f'ID {i:02d}: invalid torque state after hold goal write')
        # Only this transition is accepted, after OUR acknowledged/read-back
        # goal write. No blanket tolerance for unexpected torque changes.
        expected[i]['torque_enable'] = row['torque_enable']
        # Goal processing/activation may be transient here. Only stationarity
        # is deferred; invalid state, pose/config/goal/health still abort now.
        check_row(i, row, expected[i], poses[i], require_stationary=False)
        if row['torque_enable'] == 0:
            event('before_torque_activation', id=i, position=row)
            checked_write(packet, port, i, 24, 1, event)
            expected[i]['torque_enable'] = 1
            row = capture_sample(packet, port, i, extended=True)
            event('after_torque_activation', id=i, position=row)
            observation = 'on_after_explicit_torque_write'
        else:
            # Temporal observation, NOT proof of firmware causality or of
            # exclusive access. Position/configuration still checked above.
            observation = 'on_observed_after_hold_goal_write'
        row = wait_for_hold_settle(packet, port, i, row, expected[i], poses[i], event,
                                   timeout=settle_timeout)
        activation_observations[i] = observation
        event('torque_on_verified', id=i, observation=observation)
        event('activated_joint_feedback', id=i, position=row)

    final = read_all(packet, port)
    event('final_torque_readback_snapshot', positions=final)
    for i in IDS:
        check_row(i, final[i], expected[i], poses[i])
    event('all_torque_on_final_readback', positions=final)
    return {'status': 'ALL_TORQUE_ON_VERIFIED_CURRENT_POSE', 'positions': final,
            'newly_enabled_ids': off_ids, 'hold_goals': poses,
            'activation_observations': activation_observations,
            'hold_settling': {'timeout_s': settle_timeout, 'poll_interval_s': SETTLE_POLL_INTERVAL,
                              'required_stable_samples': SETTLE_STABLE_SAMPLES},
            'scope': 'holding preparation only; NOT Centerized pose or geometric calibration'}


def show(rows):
    for i in IDS:
        row = rows[i]
        print(f"ID {i:02d} {JOINT_NAMES[i-1]}: torque={row['torque_enable']} "
              f"present={row['present_ticks']} goal={row['goal_ticks']} "
              f"torque_limit={row['torque_limit']} speed={row['moving_speed_raw']}")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--inspect', action='store_true')
    modes.add_argument('--execute', action='store_true')
    parser.add_argument('--robot-supported', action='store_true')
    parser.add_argument('--exclusive-port-confirmed', action='store_true')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--speed-cap', type=int, default=DEFAULT_SPEED_CAP)
    parser.add_argument('--hold-settle-timeout', type=float, default=DEFAULT_SETTLE_TIMEOUT,
                        help='Seconds allowed per newly activated joint (0.1..10, default 2)')
    parser.add_argument('--output', type=Path)
    options = parser.parse_args(args)
    if options.baud <= 0 or not 1 <= options.speed_cap <= 1023:
        parser.error('Positive baud and nonzero speed cap 1..1023 required')
    if not .1 <= options.hold_settle_timeout <= 10:
        parser.error('--hold-settle-timeout must be finite and within 0.1..10 seconds')
    if options.execute and not (options.robot_supported and options.exclusive_port_confirmed):
        parser.error('--execute requires --robot-supported and --exclusive-port-confirmed')
    if not options.execute and not options.inspect:
        print('OFFLINE PREVIEW ONLY. No SDK, serial port or output files opened.')
        print('IDs 1..20; legacy MX profile; preserve existing Torque Limit and ON servo goals.')
        print(f'OFF servos: speed cap={options.speed_cap}, hold-current-pose goal, verify ON or explicitly enable.')
        print('Goal writes may activate torque. Each joint is verified before the next; explicit ON only if needed.')
        print('Use --inspect first. Enabling torque can cause motion; mechanical support required.')
        return 0

    destination = options.output or Path('torque_hold_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    destination.mkdir(parents=True, exist_ok=False)
    meta = snapshot_configuration(destination)
    shutil.copy2(Path(__file__), destination/'PrepareTorqueHold.py')
    meta.update({'arguments': {**vars(options), 'output': str(destination)},
                 'started_utc': datetime.now(timezone.utc).isoformat(),
                 'source_sha256_this_script': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 'operation': 'execute' if options.execute else 'read_only_inspect',
                 'protocol': 1.0, 'completed': False, 'read_retries': [],
                 'torque_on_attempted_ids': [], 'torque_on_verified_ids': [],
                 'hold_goal_activation_possible_ids': [], 'torque_activation_observations': {},
                 'eeprom_writes': False, 'torque_limit_writes': False,
                 'offset_writes': False, 'sequential_reads_not_atomic': True,
                 'shutdown': 'NO automatic torque-off or rollback; energized/unknown states remain'})
    port = None
    exit_code = 0
    with (destination/'events.jsonl').open('x', encoding='utf-8') as log:
        def event(kind, **values):
            log.write(json.dumps({'utc': datetime.now(timezone.utc).isoformat(),
                                  'monotonic_s': time.monotonic(), 'event': kind, **values}, allow_nan=False)+'\n')
            log.flush()
            if kind == 'write_attempt' and values['address'] == 24:
                meta['torque_on_attempted_ids'].append(values['id'])
            if kind == 'hold_goal_activation_possible':
                meta['hold_goal_activation_possible_ids'].append(values['id'])
            if kind == 'torque_on_verified':
                meta['torque_on_verified_ids'].append(values['id'])
                meta['torque_activation_observations'][str(values['id'])] = values['observation']
                print(f"ID {values['id']:02d}: torque ON verified ({values['observation'].replace('_', ' ')})")
        def observe_retry(details):
            meta['read_retries'].append(details)
            event('read_retry', **details)
            print(f"READ RETRY: ID {details['id']:02d} address={details['address']} attempt={details['attempt']}")
        try:
            from dynamixel_sdk import PortHandler, PacketHandler
            port = PortHandler(options.port)
            packet = PacketHandler(1.0)
            packet.read_retry_observer = observe_retry
            if not port.openPort() or not port.setBaudRate(options.baud):
                raise RuntimeError('Cannot open/configure U2D2 port')
            baseline = read_all(packet, port)
            meta['initial_positions'] = baseline
            event('initial_read_only_snapshot', positions=baseline)
            show(baseline)
            errors = preflight(baseline)
            meta['preflight_errors'] = errors
            if options.inspect:
                meta['status'] = 'READ_ONLY_INSPECTION_COMPLETE'
                print('Execute blockers:', errors)
            else:
                if errors:
                    raise RuntimeError('Preflight refused torque activation: '+'; '.join(errors))
                print('WARNING: Torque activation can move loaded joints. Support the robot and keep power cutoff ready.')
                print('Preserving existing torque limits, which may be 1023 (maximum). No universal safe limit is assumed.')
                print('OFF servo goals/speeds WILL change to hold the captured pose. ON servo goals/speeds remain unchanged.')
                print('A hold-goal write may activate torque; each joint is checked before preparing the next.')
                print('Close bridge/Wizard/other writers. Acknowledgement is NOT an OS-enforced port lock.')
                if input('Type HOLD CURRENT POSE AND ENABLE TORQUE to proceed: ').strip() != 'HOLD CURRENT POSE AND ENABLE TORQUE':
                    raise RuntimeError('Operator did not confirm; no writes')
                result = prepare_hold(packet, port, baseline, event, speed_cap=options.speed_cap,
                                      settle_timeout=options.hold_settle_timeout)
                meta.update(result)
                print('All 20 torque states verified ON. Current pose hold only; Centerized reference NOT reached by this tool.')
            meta['completed'] = True
        except KeyboardInterrupt:
            meta['status'] = 'INTERRUPTED_PARTIAL_STATE_MAY_REMAIN'; exit_code = 130
            print('Interrupted. Ctrl+C is NOT torque-off or an emergency stop.')
        except Exception as exc:
            meta['status'] = 'FAILED_OR_REFUSED'; meta['error'] = str(exc); exit_code = 1
            print('Stopped:', exc)
        finally:
            if port is not None:
                port.closePort()
            meta['finished_utc'] = datetime.now(timezone.utc).isoformat()
            (destination/'torque_hold_metadata.json').write_text(json.dumps(meta, indent=2, allow_nan=False)+'\n')
            print('No automatic rollback/torque-off. Existing and attempted states/goals may remain active.')
            print('Hold-goal writes that may activate torque, attempted IDs:', meta['hold_goal_activation_possible_ids'])
            print('Explicit torque-on write attempted IDs:', meta['torque_on_attempted_ids'])
            print('New torque-on verified IDs:', meta['torque_on_verified_ids'])
            print('Log folder:', destination.resolve())
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
