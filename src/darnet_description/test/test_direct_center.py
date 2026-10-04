"""Offline direct-mode tests: fake SDK only; no ROS/hardware access."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

SOURCE = Path(__file__).resolve().parents[1]/'darnet_description'
sys.path.insert(0, str(SOURCE))
from test_centerized_reference import Clock, FakePacket, baseline
from test_torque_hold import GoalActivatingPacket, Packet, rows, hold
import CenterizedReference as reference


class DirectTests(unittest.TestCase):
    def test_large_correction_is_explicit_not_default(self):
        original = baseline(200)
        self.assertTrue(any('too far' in error for error in reference.preflight(original)))
        self.assertEqual(reference.preflight(original, max_start_delta=None), [])

    def test_large_mode_does_not_bypass_limits_or_model_or_feedback(self):
        for field, value in (('model_number', 999), ('torque_enable', 0),
                             ('cw_limit', 2500), ('registered_instruction', 1),
                             ('moving', 1), ('voltage_raw', 20), ('temperature_c', 70)):
            original = baseline(200); original[1][field] = value
            with self.subTest(field=field):
                self.assertTrue(reference.preflight(original, max_start_delta=None))

    def test_large_ramp_remains_incremental_and_acknowledged(self):
        original = baseline(200); packet = FakePacket(original); clock = Clock()
        result = reference.execute_small_correction(packet, object(), original,
            lambda *a, **k: None, max_start_delta=None, sleep=clock.sleep, clock=clock.now)
        self.assertEqual(result['status'], 'REFERENCE_GOALS_REACHED_FEEDBACK_NEAR_TARGET')
        for i in reference.TARGETS:
            values = [value for sid, address, value in packet.writes if sid == i and address == 30]
            self.assertEqual(values[-1], reference.TARGETS[i])
            self.assertTrue(all(abs(a-b) <= 4 for a, b in zip([original[i]['goal_ticks']]+values, values)))

    def test_direct_requires_support_and_exclusive_acknowledgements(self):
        for args in (['--direct'], ['--direct', '--robot-supported'],
                     ['--direct', '--exclusive-port-confirmed']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                reference.main(args)
            self.assertEqual(exc.exception.code, 2)

    def run_cli(self, *, bad_limits=False, unknown_model=False, activation_timeout=False,
                goal_activates=False, goal_timeout=False, transient_moving=False):
        initial = rows()
        for i, row in initial.items():
            row['present_ticks'] = reference.TARGETS[i]+200
            row['goal_ticks'] = row['present_ticks']
        if bad_limits:
            initial[1]['ccw_limit'] = 1000
        if unknown_model:
            initial[1]['model_number'] = 999
        packet = (GoalActivatingPacket if goal_activates else Packet)(initial); port = Mock()
        port.openPort.return_value = True; port.setBaudRate.return_value = True
        if goal_timeout:
            original_write = packet.write2ByteTxRx
            def uncertain_goal(serial_port, i, address, value):
                result = original_write(serial_port, i, address, value)
                return (-3001, 0) if address == 30 else result
            packet.write2ByteTxRx = uncertain_goal
        if activation_timeout:
            def enable(serial_port, i, address, value):
                if i == 2:
                    packet.rows[i]['torque_enable'] = 1
                    return -3001, 0
                return FakePacket.write2ByteTxRx(packet, serial_port, i, address, value)
            packet.write1ByteTxRx = enable
        sdk = types.SimpleNamespace(PortHandler=lambda path: port, PacketHandler=lambda protocol: packet)
        clock = Clock(); original_execute = reference.execute_small_correction
        actual_wait = hold.wait_for_hold_settle
        def fast_wait(*args, **kwargs):
            if transient_moving:
                args[3]['moving'] = 1  # Initial post-activation row only.
            return actual_wait(*args, **kwargs, sleep=clock.sleep, clock=clock.now)
        def offline_execute(*args, **kwargs):
            return original_execute(*args, **kwargs, sleep=clock.sleep, clock=clock.now)
        with tempfile.TemporaryDirectory() as temp:
            destination = Path(temp)/'run'
            with patch.dict(sys.modules, {'dynamixel_sdk': sdk}), \
                 patch.object(reference, 'snapshot_configuration', return_value={'configuration_errors': []}), \
                 patch.object(reference, 'execute_small_correction', side_effect=offline_execute), \
                 patch.object(hold, 'wait_for_hold_settle', side_effect=fast_wait), \
                 patch('builtins.input', side_effect=AssertionError('Direct mode must not prompt')), \
                 contextlib.redirect_stdout(io.StringIO()):
                rc = reference.main(['--direct', '--robot-supported', '--exclusive-port-confirmed',
                                     '--output', str(destination)])
            meta = json.loads((destination/'reference_metadata.json').read_text())
            events = [json.loads(line) for line in (destination/'events.jsonl').read_text().splitlines()]
        return rc, packet, port, meta, events

    def test_direct_cli_full_torque_then_large_correction_without_prompt(self):
        rc, packet, port, meta, events = self.run_cli()
        self.assertEqual(rc, 0); self.assertTrue(meta['completed'])
        self.assertTrue(meta['torque_enable_writes'])
        self.assertFalse(meta['interactive_confirmation'])
        self.assertIsNone(meta['max_start_delta_ticks'])
        self.assertEqual(meta['torque_on_verified_ids'], list(range(1, 21)))
        self.assertTrue(all(row['torque_enable'] == 1 for row in packet.rows.values()))
        self.assertEqual({i: row['goal_ticks'] for i, row in packet.rows.items()}, reference.TARGETS)
        kinds = [event['event'] for event in events]
        self.assertLess(kinds.index('torque_preparation_complete'), kinds.index('goal_written'))
        self.assertTrue(all(address in (24, 30, 32) for _, address, _ in packet.writes))
        port.closePort.assert_called_once()

    def test_invalid_limits_or_model_refused_before_any_write(self):
        for kwargs in ({'bad_limits': True}, {'unknown_model': True}):
            rc, packet, port, meta, events = self.run_cli(**kwargs)
            self.assertEqual(rc, 1); self.assertFalse(meta['completed'])
            self.assertEqual(packet.writes, [])
            port.closePort.assert_called_once()

    def test_partial_torque_timeout_aborts_centering_and_records_unknown(self):
        rc, packet, port, meta, events = self.run_cli(activation_timeout=True)
        self.assertEqual(rc, 1); self.assertFalse(meta['completed'])
        self.assertEqual(meta['torque_on_attempted_ids'], [1, 2])
        self.assertEqual(meta['torque_on_verified_ids'], [1])
        self.assertFalse(any(event['event'] == 'goal_written' for event in events))
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertEqual(packet.rows[2]['torque_enable'], 1)
        self.assertEqual(packet.rows[3]['torque_enable'], 0)

    def test_goal_activation_full_direct_cli_no_prompt_or_explicit_torque_write(self):
        rc, packet, port, meta, events = self.run_cli(goal_activates=True)
        self.assertEqual(rc, 0); self.assertTrue(meta['completed'])
        self.assertEqual(meta['torque_on_attempted_ids'], [])
        self.assertEqual(meta['hold_goal_activation_possible_ids'], list(range(1, 21)))
        self.assertEqual(meta['torque_on_verified_ids'], list(range(1, 21)))
        self.assertFalse(any(address == 24 for _, address, _ in packet.writes))
        self.assertEqual({i: row['goal_ticks'] for i, row in packet.rows.items()}, reference.TARGETS)
        self.assertEqual(set(meta['torque_activation_observations'].values()),
                         {'on_observed_after_hold_goal_write'})

    def test_unknown_goal_activation_logged_and_centering_not_started(self):
        rc, packet, port, meta, events = self.run_cli(goal_activates=True, goal_timeout=True)
        self.assertEqual(rc, 1); self.assertFalse(meta['completed'])
        self.assertEqual(meta['hold_goal_activation_possible_ids'], [1])
        self.assertEqual(meta['torque_on_attempted_ids'], [])
        self.assertEqual(meta['torque_on_verified_ids'], [])
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertEqual(packet.rows[2]['torque_enable'], 0)
        self.assertFalse(any(event['event'] == 'goal_written' for event in events))

    def test_direct_transient_activation_moving_reaches_final_reference(self):
        rc, packet, port, meta, events = self.run_cli(goal_activates=True, transient_moving=True)
        self.assertEqual(rc, 0); self.assertTrue(meta['completed'])
        self.assertEqual({i: row['goal_ticks'] for i, row in packet.rows.items()}, reference.TARGETS)
        self.assertEqual(len([e for e in events if e['event'] == 'hold_settled']), 20)
        self.assertEqual(meta['torque_preparation']['hold_settling']['timeout_s'], 2)


if __name__ == '__main__':
    unittest.main()
