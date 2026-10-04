#!/usr/bin/env python3
"""Read-only MX Protocol 1.0 capture; never enables/disables torque or moves joints."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time


# These addresses are ONLY for the legacy MX-28 / MX-64 control tables.
SUPPORTED_MODELS = {29: 'MX-28', 310: 'MX-64'}
REGISTERS = (
    ('cw_limit', 6, 2), ('ccw_limit', 8, 2),
    ('resolution_divider', 22, 1), ('torque_enable', 24, 1),
    ('goal_ticks', 30, 2), ('present_ticks', 36, 2),
    ('voltage_raw', 42, 1), ('temperature_c', 43, 1),
)

# Additional raw configuration evidence, never written by this reader.
EXTENDED_REGISTERS = (
    ('firmware_version', 2, 1), ('device_id', 3, 1), ('baud_register', 4, 1),
    ('return_delay', 5, 1), ('temperature_limit_c', 11, 1),
    ('voltage_min_raw', 12, 1), ('voltage_max_raw', 13, 1),
    ('max_torque', 14, 2), ('status_return_level', 16, 1),
    ('multiturn_offset_raw', 20, 2), ('d_gain', 26, 1), ('i_gain', 27, 1),
    ('p_gain', 28, 1), ('moving_speed_raw', 32, 2), ('torque_limit', 34, 2),
    ('present_speed_raw', 38, 2), ('present_load_raw', 40, 2),
    ('registered_instruction', 44, 1), ('moving', 46, 1),
)


def infer_mode(row):
    """Legacy MX mode inferred from limits, with MX-64 torque mode override."""
    if row.get('torque_control_mode') == 1:
        return 'torque_control'
    lo, hi = row['cw_limit'], row['ccw_limit']
    if lo == hi == 0:
        return 'wheel'
    if lo == hi == 4095:
        return 'multiturn'
    if 0 <= lo < hi <= 4095:
        return 'joint'
    return 'invalid_or_unrecognized'


def read_value(packet, port, servo_id, address, size, *, attempts=3, retry_delay=.02):
    """Retry only READ timeout/corrupt responses; never retry any motor write.

    Registers are sequential reads, not an atomic snapshot. Successful retries
    can be recorded using packet.read_retry_observer(details). Device errors,
    port-busy and transmit failures abort immediately. No SDK import needed.
    """
    if size not in (1, 2) or not 1 <= attempts <= 5 or not 0 <= retry_delay <= 1:
        raise ValueError('Invalid read size/retry settings')
    method = packet.read1ByteTxRx if size == 1 else packet.read2ByteTxRx
    for attempt in range(1, attempts+1):
        value, result, error = method(port, servo_id, address)
        prefix = f'ID {servo_id:02d} READ address={address} size={size} attempt={attempt}/{attempts}: '
        if error:
            raise RuntimeError(prefix + packet.getRxPacketError(error))
        if result == 0:
            return value
        message = packet.getTxRxResult(result)
        if result not in (-3001, -3002) or attempt == attempts:
            raise RuntimeError(prefix + message)
        observer = getattr(packet, 'read_retry_observer', None)
        if callable(observer):
            observer({'id': servo_id, 'address': address, 'size': size,
                      'attempt': attempt, 'result': result, 'error': message})
        time.sleep(retry_delay)


def capture_sample(packet, port, servo_id, extended=False):
    """Check model before touching model-dependent addresses; do not decode angles."""
    model = read_value(packet, port, servo_id, 0, 2)
    row = {'model_number': model, 'model': SUPPORTED_MODELS.get(model, 'unsupported')}
    if model not in SUPPORTED_MODELS:
        raise RuntimeError(f'ID {servo_id:02d}: Unsupported model {model}; no legacy registers read')
    # One contiguous block instead of one round-trip for every register.
    # Model is checked first; no MX-specific addresses on an unsupported model.
    data = read_block(packet, port, servo_id, 2, 45)
    for name, address, size in REGISTERS:
        row[name] = decode_register(data, 2, address, size)
    if extended:
        for name, address, size in EXTENDED_REGISTERS:
            row[name] = decode_register(data, 2, address, size)
        # Address 70 is MX-64-specific; do not read it on MX-28.
        if model == 310:
            row['torque_control_mode'] = read_value(packet, port, servo_id, 70, 1)
        row['operating_mode_inferred'] = infer_mode(row)
    return row


def read_block(packet, port, servo_id, address, length, *, attempts=3, retry_delay=.02):
    """Bounded READ-only retries; exact-length validated byte array.

    A register block reduces USB round-trips, not a claim of an atomic sample.
    No broadcast READ and no automatic WRITE retries.
    """
    if (type(address) is not int or type(length) is not int or
            not 0 <= address <= 255 or not 1 <= length <= 200 or address+length > 256 or
            not 1 <= attempts <= 5 or not 0 <= retry_delay <= 1):
        raise ValueError('Invalid block read settings')
    for attempt in range(1, attempts+1):
        data, result, error = packet.readTxRx(port, servo_id, address, length)
        prefix = f'ID {servo_id:02d} BLOCK READ address={address} length={length} attempt={attempt}/{attempts}: '
        if error:
            raise RuntimeError(prefix + packet.getRxPacketError(error))
        if result == 0:
            if len(data) != length or any(type(v) is not int or not 0 <= v <= 255 for v in data):
                raise RuntimeError(prefix + 'invalid length/bytes; no data used')
            return data
        message = packet.getTxRxResult(result)
        if result not in (-3001, -3002) or attempt == attempts:
            raise RuntimeError(prefix + message)
        observer = getattr(packet, 'read_retry_observer', None)
        if callable(observer):
            observer({'id': servo_id, 'address': address, 'size': length,
                      'attempt': attempt, 'result': result, 'error': message})
        time.sleep(retry_delay)


def decode_register(data, start, address, size):
    """Legacy MX unsigned little-endian register, no unit conversion."""
    offset = address-start
    if size not in (1, 2) or offset < 0 or offset+size > len(data):
        raise ValueError('Register outside block')
    return sum(data[offset+n] << (8*n) for n in range(size))


def snapshot_configuration(destination):
    """Copy resolved installed files without importing/starting the motor bridge."""
    metadata = {'resolved_files': {}, 'source_sha256': {}, 'configuration_errors': [],
                'python_executable': sys.executable, 'python_version': sys.version,
                'ros_distro': os.environ.get('ROS_DISTRO'),
                'ament_prefix_path': os.environ.get('AMENT_PREFIX_PATH')}
    for module in ('ComsROS2U2D2', 'Centerized', 'CenterizedReference', 'CaptureEncoderReference'):
        try:
            spec = importlib.util.find_spec(f'darnet_description.{module}')
            source = Path(spec.origin).resolve()
            shutil.copy2(source, destination / f'{module}.py')
            metadata['resolved_files'][module] = str(source)
            metadata['source_sha256'][module] = hashlib.sha256(source.read_bytes()).hexdigest()
        except Exception as exc:
            metadata['configuration_errors'].append(f'{module}: {exc}')
    try:
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('darnet_description')) / 'config' / 'zero_offsets.json'
        metadata['resolved_files']['zero_offsets'] = str(source.resolve())
        shutil.copy2(source, destination / 'zero_offsets.json')
        metadata['source_sha256']['zero_offsets'] = hashlib.sha256(source.read_bytes()).hexdigest()
        metadata['offsets_status'] = 'historical_demo_reference_only_not_applied'
    except Exception as exc:
        metadata['configuration_errors'].append(f'zero_offsets: {exc}')
    return metadata


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--ids', type=int, nargs='+', default=list(range(1, 21)))
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--interval', type=float, default=0.5)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--pose-confirmed', action='store_true',
                        help='Operator confirms geometric zero pose; NOT automatic calibration')
    options = parser.parse_args(args)
    if (options.samples < 1 or not math.isfinite(options.interval) or options.interval < 0 or options.baud < 1
            or any(i < 1 or i > 253 for i in options.ids)
            or len(set(options.ids)) != len(options.ids)):
        parser.error('Invalid samples, interval, baud, or IDs (unique IDs 1..253 required)')
    destination = options.output or Path(
        'encoder_capture_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    destination.mkdir(parents=True, exist_ok=False)
    metadata = snapshot_configuration(destination)
    metadata.update({
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'port': options.port, 'baud': options.baud, 'protocol': 1.0,
        'ids': options.ids, 'samples': options.samples,
        'pose_confirmed_by_operator': options.pose_confirmed,
        'status': 'raw_candidate_not_final_calibration',
        'register_profile': 'legacy_MX_extended_raw_configuration_v2',
        'note': 'Sequential raw reads; no offsets applied. Snapshot is this process environment, not proof of previous bridge runtime.',
        'completed': False,
    })
    fields = ['timestamp_utc', 'sample', 'id', 'model_number', 'model']
    fields += [name for name, _, _ in REGISTERS] + ['pose_confirmed', 'error']
    fields += [name for name, _, _ in EXTENDED_REGISTERS]
    fields += ['torque_control_mode', 'operating_mode_inferred']
    port = None
    failures = 0
    interrupted = False
    print('READ ONLY. Stop the bridge/Wizard first. Torque and goals remain unchanged.')
    print('Keep the robot supervised; holding torque cannot guarantee balance or safety.')
    try:
        from dynamixel_sdk import PortHandler, PacketHandler
        port = PortHandler(options.port)
        packet = PacketHandler(1.0)
        metadata['read_retries'] = []
        def observe_retry(details):
            metadata['read_retries'].append({'timestamp_utc': datetime.now(timezone.utc).isoformat(), **details})
            print(f"READ RETRY: ID {details['id']:02d} address={details['address']} attempt={details['attempt']}: {details['error']}")
        packet.read_retry_observer = observe_retry
        if not port.openPort():
            raise RuntimeError(f'Cannot open {options.port}')
        if not port.setBaudRate(options.baud):
            raise RuntimeError(f'Cannot set baud {options.baud}')
        with (destination / 'encoder_reference_centerized.csv').open(
                'x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for sample in range(1, options.samples + 1):
                for servo_id in options.ids:
                    row = {'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                           'sample': sample, 'id': servo_id,
                           'pose_confirmed': options.pose_confirmed, 'error': ''}
                    try:
                        row.update(capture_sample(packet, port, servo_id, extended=True))
                    except Exception as exc:
                        failures += 1
                        row['error'] = str(exc)
                    writer.writerow(row)
                    stream.flush()
                    print(f"ID {servo_id:02d} sample {sample}: "
                          f"present={row.get('present_ticks', '?')} "
                          f"torque={row.get('torque_enable', '?')} {row['error']}")
                if sample < options.samples:
                    time.sleep(options.interval)
        metadata['completed'] = True
    except KeyboardInterrupt:
        interrupted = True
        metadata['capture_error'] = 'Interrupted by operator'
    except Exception as exc:
        failures += 1
        metadata['capture_error'] = str(exc)
        print(f'Capture failed: {exc}')
    finally:
        if port is not None:
            port.closePort()  # No torque write, including on error or Ctrl+C.
        metadata['read_failures'] = failures
        metadata['finished_utc'] = datetime.now(timezone.utc).isoformat()
        (destination / 'capture_metadata.json').write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
        print(f'Capture folder: {destination.resolve()}')
    return 130 if interrupted else (1 if failures else 0)


if __name__ == '__main__':
    raise SystemExit(main())
