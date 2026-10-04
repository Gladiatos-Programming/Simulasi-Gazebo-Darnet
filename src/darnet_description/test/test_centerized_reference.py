"""Offline mock tests; never imports ROS or the Dynamixel SDK."""
import ast
import contextlib
import copy
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[1] / 'darnet_description'
spec = importlib.util.spec_from_file_location('CaptureEncoderReference', SOURCE/'CaptureEncoderReference.py')
capture = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = capture
spec.loader.exec_module(capture)
spec = importlib.util.spec_from_file_location('CenterizedReference', SOURCE/'CenterizedReference.py')
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


def baseline(delta=12):
    return {i: {'model_number': reference.EXPECTED_MODELS[i], 'model': 'mock',
                'device_id': i, 'cw_limit': 0, 'ccw_limit': 4095,
                'resolution_divider': 1, 'multiturn_offset_raw': 0,
                'status_return_level': 2, 'torque_enable': 1, 'torque_limit': 1023,
                'registered_instruction': 0, 'moving': 0, 'present_speed_raw': 0,
                'present_ticks': target+delta, 'goal_ticks': target+delta,
                'temperature_c': 35, 'temperature_limit_c': 80,
                'voltage_raw': 120, 'voltage_min_raw': 60, 'voltage_max_raw': 160,
                'moving_speed_raw': 100, 'torque_control_mode': 0}
            for i, target in reference.TARGETS.items()}


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


class FakePacket:
    fields = {0: 'model_number', 2: 'firmware_version', 3: 'device_id', 4: 'baud_register',
              5: 'return_delay', 6: 'cw_limit', 8: 'ccw_limit', 11: 'temperature_limit_c',
              12: 'voltage_min_raw', 13: 'voltage_max_raw', 14: 'max_torque',
              16: 'status_return_level', 20: 'multiturn_offset_raw', 22: 'resolution_divider',
              24: 'torque_enable', 26: 'd_gain', 27: 'i_gain', 28: 'p_gain',
              30: 'goal_ticks', 32: 'moving_speed_raw', 34: 'torque_limit',
              36: 'present_ticks', 38: 'present_speed_raw', 40: 'present_load_raw',
              42: 'voltage_raw', 43: 'temperature_c', 44: 'registered_instruction',
              46: 'moving', 70: 'torque_control_mode'}

    def __init__(self, rows=None):
        self.rows = copy.deepcopy(rows or baseline())
        self.writes = []; self.reads = []; self.stalled = False
        self.write_error = False; self.wrong_readback = False

    def read2ByteTxRx(self, port, servo_id, address):
        self.reads.append((servo_id, address))
        value = self.rows[servo_id].get(self.fields[address], 0)
        if self.wrong_readback and self.writes and address == self.writes[-1][1]:
            value += 1
        return value, 0, 0

    read1ByteTxRx = read2ByteTxRx

    def write2ByteTxRx(self, port, servo_id, address, value):
        if self.write_error: return -1, 0
        self.writes.append((servo_id, address, value))
        self.rows[servo_id][self.fields[address]] = value
        if address == 30 and not self.stalled:
            self.rows[servo_id]['present_ticks'] = value
        return 0, 0

    def getTxRxResult(self, result): return 'mock transport'
    def getRxPacketError(self, error): return 'mock device'


class ReferenceTests(unittest.TestCase):
    def test_targets_and_mapping(self):
        self.assertEqual(len(reference.TARGETS), 20)
        self.assertEqual(reference.TARGETS[8], 1946)
        self.assertEqual(reference.TARGETS[12], 2081)
        self.assertTrue(all(v == 2048 for i, v in reference.TARGETS.items() if i not in (8, 12)))
        self.assertEqual(reference.JOINT_NAMES[7], 'Paha Kiri Putar')
        self.assertEqual(reference.JOINT_NAMES[11], 'Paha Bawah Kiri')

    def test_preview_is_offline_and_creates_no_outputs(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(reference.main([]), 0)
        self.assertIn('OFFLINE PREVIEW ONLY', out.getvalue())
        tree = ast.parse((SOURCE/'CenterizedReference.py').read_text())
        imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any(isinstance(n, ast.ImportFrom) and n.module in ('rclpy', 'dynamixel_sdk') for n in imports))

    def test_execute_missing_acknowledgement_refuses_before_hardware(self):
        for args in (['--execute'], ['--execute', '--robot-supported'],
                     ['--execute', '--exclusive-port-confirmed']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                reference.main(args)
            self.assertEqual(exc.exception.code, 2)

    def test_valid_preflight(self):
        self.assertEqual(reference.preflight(baseline()), [])

    def test_unsafe_preflight_cases(self):
        cases = (('model_number', 999), ('device_id', 99), ('torque_enable', 0),
                 ('torque_limit', 0), ('resolution_divider', 2), ('multiturn_offset_raw', 1),
                 ('status_return_level', 1), ('registered_instruction', 1), ('moving', 1),
                 ('present_speed_raw', 100), ('temperature_c', 60), ('voltage_raw', 20),
                 ('torque_control_mode', 1), ('cw_limit', 2100))
        for field, value in cases:
            rows = baseline(); rows[8][field] = value
            with self.subTest(field=field): self.assertTrue(reference.preflight(rows))
        for lo, hi in ((0, 0), (4095, 4095), (3000, 2000)):
            rows = baseline(); rows[8].update(cw_limit=lo, ccw_limit=hi)
            self.assertTrue(reference.preflight(rows))

    def test_missing_id_or_distant_pose(self):
        rows = baseline(); del rows[20]
        self.assertTrue(reference.preflight(rows))
        self.assertTrue(reference.preflight(baseline(200)))
        rows = baseline(); rows[1]['present_ticks'] -= 40
        self.assertTrue(reference.preflight(rows))

    def test_ramp_monotonic_and_step_bounded(self):
        for delta in (-128, -12, 12, 128):
            goals = {i: t+delta for i, t in reference.TARGETS.items()}
            for _ in range(32):
                next_goals = reference.next_frame(goals)
                for i in goals:
                    self.assertLessEqual(abs(next_goals[i]-goals[i]), 4)
                    self.assertLessEqual(abs(next_goals[i]-reference.TARGETS[i]), abs(goals[i]-reference.TARGETS[i]))
                goals = next_goals
            self.assertEqual(goals, reference.TARGETS)

    def test_guarded_success_speed_first_and_no_torque_or_eeprom_writes(self):
        packet = FakePacket(); clock = Clock(); events = []
        result = reference.execute_small_correction(packet, object(), baseline(),
            lambda kind, **values: events.append(kind), sleep=clock.sleep, clock=clock.now)
        self.assertEqual(result['status'], 'REFERENCE_GOALS_REACHED_FEEDBACK_NEAR_TARGET')
        self.assertEqual([a for _, a, _ in packet.writes[:20]], [32]*20)
        self.assertEqual({a for _, a, _ in packet.writes}, {30, 32})
        self.assertTrue(all(v == 20 for _, a, v in packet.writes if a == 32))
        self.assertEqual(sum(e == 'settling_feedback' for e in events), 3)
        for i in reference.TARGETS:
            goals = [v for sid, a, v in packet.writes if sid == i and a == 30]
            self.assertEqual(goals[-1], reference.TARGETS[i])
            self.assertTrue(all(abs(a-b) <= 4 for a, b in zip([baseline()[i]['goal_ticks']]+goals, goals)))

    def test_already_target_performs_no_writes(self):
        rows = baseline(0); packet = FakePacket(rows)
        result = reference.execute_small_correction(packet, object(), rows, lambda *a, **k: None)
        self.assertEqual(result['status'], 'ALREADY_AT_REFERENCE_GOALS_NO_WRITES')
        self.assertEqual(packet.writes, [])

    def test_bad_preflight_has_no_writes(self):
        rows = baseline(); rows[1]['torque_enable'] = 0; packet = FakePacket(rows)
        with self.assertRaises(RuntimeError):
            reference.execute_small_correction(packet, object(), rows, lambda *a, **k: None)
        self.assertEqual(packet.writes, [])

    def test_stall_aborts_further_goals_without_disabling_torque(self):
        rows = baseline(100); packet = FakePacket(rows); packet.stalled = True; clock = Clock()
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            reference.execute_small_correction(packet, object(), rows, lambda *a, **k: None,
                                               sleep=clock.sleep, clock=clock.now)
        self.assertTrue(all(a in (30, 32) for _, a, _ in packet.writes))
        self.assertNotEqual(packet.rows[1]['goal_ticks'], reference.TARGETS[1])

    def test_write_failure_and_readback_mismatch(self):
        for flag in ('write_error', 'wrong_readback'):
            packet = FakePacket(); setattr(packet, flag, True)
            with self.assertRaises(RuntimeError): reference.checked_write(packet, object(), 1, 32, 20)

    def test_write_allowlist(self):
        packet = FakePacket()
        for address, value in ((24, 1), (34, 1023), (6, 0), (30, 5000), (32, 0), (30, True)):
            with self.assertRaises(ValueError): reference.checked_write(packet, object(), 1, address, value)
        self.assertEqual(packet.writes, [])

    def test_runtime_competing_writer_or_mode_changes(self):
        rows = baseline(); goals = {i: r['goal_ticks'] for i, r in rows.items()}
        for field, value in (('goal_ticks', 0), ('moving_speed_raw', 0),
                             ('cw_limit', 1), ('torque_enable', 0), ('torque_control_mode', 1)):
            packet = FakePacket(rows)
            for r in packet.rows.values(): r['moving_speed_raw'] = 20
            packet.rows[8][field] = value
            with self.assertRaises(RuntimeError): reference.check_runtime(packet, object(), rows, goals)

    def test_extended_reader_model_specific_registers_and_mode(self):
        packet = FakePacket()
        a = capture.capture_sample(packet, object(), 1, extended=True)
        b = capture.capture_sample(packet, object(), 8, extended=True)
        self.assertEqual(a['operating_mode_inferred'], 'joint')
        self.assertIn('p_gain', a)
        self.assertNotIn((1, 70), packet.reads)
        self.assertIn((8, 70), packet.reads)
        self.assertEqual(b['torque_control_mode'], 0)
        self.assertEqual(packet.writes, [])

    def test_mode_inference(self):
        for lo, hi, mode in ((0, 0, 'wheel'), (4095, 4095, 'multiturn'),
                             (0, 4095, 'joint'), (3000, 1000, 'invalid_or_unrecognized')):
            self.assertEqual(capture.infer_mode({'cw_limit': lo, 'ccw_limit': hi}), mode)

    def run_cli_mock(self, args, *, confirm='MOVE CENTERIZED', snapshots=None, interrupted=False):
        packet = FakePacket(); port = Mock()
        port.openPort.return_value = True; port.setBaudRate.return_value = True
        sdk = types.SimpleNamespace(PortHandler=lambda path: port, PacketHandler=lambda protocol: packet)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'new_capture'
            with patch.dict(sys.modules, {'dynamixel_sdk': sdk}), \
                 patch.object(reference, 'snapshot_configuration', return_value={'configuration_errors': []}), \
                 patch.object(reference, 'read_all', side_effect=snapshots or [baseline(), baseline()]), \
                 patch('builtins.input', side_effect=KeyboardInterrupt if interrupted else None, return_value=confirm), \
                 contextlib.redirect_stdout(io.StringIO()):
                rc = reference.main(args+['--output', str(output)])
            import json
            meta = json.loads((output/'reference_metadata.json').read_text())
        return rc, packet, port, meta

    def test_inspection_lifecycle_never_writes(self):
        rc, packet, port, meta = self.run_cli_mock(['--inspect'])
        self.assertEqual(rc, 0); self.assertTrue(meta['completed'])
        self.assertEqual(packet.writes, []); port.closePort.assert_called_once()

    def test_operator_cancellation_and_ctrl_c_preserve_no_write(self):
        args = ['--execute', '--robot-supported', '--exclusive-port-confirmed']
        for interrupted in (False, True):
            rc, packet, port, meta = self.run_cli_mock(args, confirm='NO', interrupted=interrupted)
            self.assertEqual(rc, 130 if interrupted else 1)
            self.assertFalse(meta['completed']); self.assertEqual(packet.writes, [])
            port.closePort.assert_called_once()

    def test_robot_moves_during_prompt_no_write(self):
        fresh = baseline(); fresh[1]['present_ticks'] += 10
        rc, packet, port, meta = self.run_cli_mock(
            ['--execute', '--robot-supported', '--exclusive-port-confirmed'], snapshots=[baseline(), fresh])
        self.assertEqual(rc, 1); self.assertIn('moved during confirmation', meta['error'])
        self.assertEqual(packet.writes, []); port.closePort.assert_called_once()


if __name__ == '__main__':
    unittest.main()
