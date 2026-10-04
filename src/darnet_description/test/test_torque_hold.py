"""Offline safety / retry tests only. No hardware SDK, ROS or serial connection."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

SOURCE = Path(__file__).resolve().parents[1] / 'darnet_description'
sys.path.insert(0, str(SOURCE))
from test_centerized_reference import Clock, FakePacket, baseline
import CaptureEncoderReference as capture
import PrepareTorqueHold as hold


def rows(off_ids=range(1, 21)):
    result = baseline()
    for i, row in result.items():
        row['max_torque'] = 1023
        row['torque_enable'] = 0 if i in off_ids else 1
        row.update(baud_register=1, return_delay=250, firmware_version=36,
                   p_gain=32, i_gain=0, d_gain=0)
        if row['model_number'] != 310:
            row.pop('torque_control_mode', None)
    return result


class Packet(FakePacket):
    write1ByteTxRx = FakePacket.write2ByteTxRx


class GoalActivatingPacket(Packet):
    """Synthetic servo: a goal update turns torque on. Not a firmware claim."""
    def write2ByteTxRx(self, port, servo_id, address, value):
        result = super().write2ByteTxRx(port, servo_id, address, value)
        if address == 30 and result == (0, 0):
            self.rows[servo_id]['torque_enable'] = 1
        return result


class HoldTests(unittest.TestCase):
    def setUp(self):
        # Synthetic clock: offline tests never spend real time waiting on a servo.
        actual_wait = hold.wait_for_hold_settle
        clock = Clock()
        def fast_wait(*args, **kwargs):
            kwargs.setdefault('sleep', clock.sleep)
            kwargs.setdefault('clock', clock.now)
            return actual_wait(*args, **kwargs)
        patcher = patch.object(hold, 'wait_for_hold_settle', side_effect=fast_wait)
        patcher.start(); self.addCleanup(patcher.stop)

    def test_default_preview_no_files_or_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)/'not_created'
            with patch.dict(sys.modules, {'dynamixel_sdk': None}), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(hold.main(['--output', str(destination)]), 0)
            self.assertFalse(destination.exists())

    def test_execute_requires_both_acknowledgements(self):
        for arguments in (['--execute'], ['--execute', '--robot-supported'],
                          ['--execute', '--exclusive-port-confirmed']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                hold.main(arguments)

    def test_success_handles_one_joint_completely_before_next(self):
        original = rows(); packet = Packet(original); events = []
        result = hold.prepare_hold(packet, object(), original,
                                   lambda kind, **values: events.append((kind, values)))
        self.assertEqual(result['status'], 'ALL_TORQUE_ON_VERIFIED_CURRENT_POSE')
        self.assertEqual(len(result['newly_enabled_ids']), 20)
        self.assertEqual([address for _, address, _ in packet.writes], [32, 30, 24]*20)
        self.assertEqual([i for i, _, _ in packet.writes], [i for i in hold.IDS for _ in range(3)])
        self.assertEqual({address for _, address, _ in packet.writes}, {24, 30, 32})
        self.assertTrue(all(row['torque_enable'] == 1 for row in result['positions'].values()))
        self.assertEqual([value for _, address, value in packet.writes if address == 30],
                         [original[i]['present_ticks'] for i in hold.IDS])

    def test_on_goals_and_speed_untouched_and_limits_preserved(self):
        original = rows(off_ids=[2, 3]); packet = Packet(original)
        result = hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
        self.assertEqual({i for i, _, _ in packet.writes}, {2, 3})
        self.assertEqual(result['positions'][1]['goal_ticks'], original[1]['goal_ticks'])
        self.assertEqual(result['positions'][1]['moving_speed_raw'], original[1]['moving_speed_raw'])
        self.assertTrue(all(row['torque_limit'] == 1023 for row in result['positions'].values()))

    def test_all_already_on_no_writes(self):
        original = rows(off_ids=[]); packet = Packet(original)
        hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
        self.assertEqual(packet.writes, [])

    def test_invalid_preflight_never_writes(self):
        cases = (('torque_limit', 0), ('status_return_level', 1), ('device_id', 33),
                 ('model_number', 999), ('moving', 1), ('present_speed_raw', 50),
                 ('voltage_raw', 20), ('temperature_c', 70), ('registered_instruction', 1),
                 ('resolution_divider', 2), ('multiturn_offset_raw', 1), ('torque_enable', 2))
        for field, value in cases:
            original = rows(); original[1][field] = value; packet = Packet(original)
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
            self.assertEqual(packet.writes, [])
        original = rows(); del original[20]
        with self.assertRaises(RuntimeError):
            hold.prepare_hold(Packet(original), object(), original, lambda *a, **k: None)

    def test_off_old_goal_replaced_but_distant_on_goal_rejected(self):
        original = rows(); original[1]['goal_ticks'] = 1000
        packet = Packet(original)
        result = hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
        self.assertEqual(result['positions'][1]['goal_ticks'], original[1]['present_ticks'])
        original[1]['torque_enable'] = 1
        self.assertTrue(hold.preflight(original))

    def test_drift_or_config_change_after_prompt_no_writes(self):
        for field, delta in (('present_ticks', 9), ('torque_limit', -1),
                             ('torque_enable', 1)):
            original = rows(); packet = Packet(original)
            packet.rows[1][field] += delta
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
            self.assertEqual(packet.writes, [])

    def test_unowned_off_goal_change_before_preload_is_logged_and_replaced(self):
        original = rows(); packet = Packet(original)
        packet.rows[2]['goal_ticks'] += 1
        events = []
        result = hold.prepare_hold(packet, object(), original,
                                   lambda kind, **values: events.append((kind, values)))
        self.assertEqual(result['positions'][2]['goal_ticks'], original[2]['present_ticks'])
        observed = [values for kind, values in events if kind == 'off_goal_changed_before_preload']
        self.assertEqual(observed[0]['id'], 2)
        self.assertEqual(observed[0]['observed'], original[2]['goal_ticks']+1)

    def test_already_on_goal_change_remains_rejected_with_values(self):
        original = rows(off_ids=[]); packet = Packet(original)
        packet.rows[2]['goal_ticks'] += 1
        with self.assertRaisesRegex(RuntimeError, r'goal_ticks changed: previous=\d+, observed=\d+'):
            hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
        self.assertEqual(packet.writes, [])

    def test_goal_change_after_preload_remains_rejected_before_enable(self):
        original = rows(); packet = Packet(original)
        def event(kind, **values):
            if kind == 'write_verified' and values['id'] == 1 and values['address'] == 30:
                packet.rows[1]['goal_ticks'] += 1
        with self.assertRaisesRegex(RuntimeError, 'goal_ticks changed'):
            hold.prepare_hold(packet, object(), original, event)
        self.assertFalse(any(address == 24 for _, address, _ in packet.writes))

    def test_off_goal_exception_does_not_hide_pose_or_config_drift(self):
        original = rows()
        for field, delta in (('present_ticks', 9), ('torque_enable', 1), ('torque_limit', -1)):
            observed = dict(original[2]); observed['goal_ticks'] += 1
            observed[field] += delta
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                hold.check_row(2, observed, original[2], original[2]['present_ticks'], check_goal=False)

    def test_drift_after_preload_aborts_before_torque_on(self):
        original = rows(); packet = Packet(original)
        def event(kind, **values):
            if kind == 'after_hold_goal_write' and values['id'] == 1:
                packet.rows[1]['present_ticks'] += 9
                values['position']['present_ticks'] += 9
        with self.assertRaisesRegex(RuntimeError, 'pose drift'):
            hold.prepare_hold(packet, object(), original, event)
        self.assertFalse(any(address == 24 for _, address, _ in packet.writes))

    def test_timeout_after_write_no_retry_and_records_attempt(self):
        packet = Packet(rows()); packet.write1ByteTxRx = Mock(return_value=(-3001, 0))
        events = []
        with self.assertRaisesRegex(RuntimeError, 'NO automatic write retry'):
            hold.checked_write(packet, object(), 1, 24, 1,
                               lambda kind, **values: events.append((kind, values)))
        packet.write1ByteTxRx.assert_called_once()
        self.assertEqual(events[0][0], 'write_attempt')

    def test_bad_feedback_after_enable_stops_other_activations(self):
        original = rows(); packet = Packet(original)
        def event(kind, **values):
            if kind == 'after_torque_activation' and values['id'] == 1:
                packet.rows[1]['present_ticks'] += 9
                values['position']['present_ticks'] += 9
        with self.assertRaisesRegex(RuntimeError, 'pose drift'):
            hold.prepare_hold(packet, object(), original, event)
        self.assertEqual([i for i, address, _ in packet.writes if address == 24], [1])
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertEqual(packet.rows[2]['torque_enable'], 0)

    def test_goal_activating_servo_needs_no_explicit_on_write(self):
        original = rows(); packet = GoalActivatingPacket(original); events = []
        result = hold.prepare_hold(packet, object(), original,
                                   lambda kind, **values: events.append((kind, values)))
        self.assertEqual([address for _, address, _ in packet.writes], [32, 30]*20)
        self.assertTrue(all(row['torque_enable'] == 1 for row in result['positions'].values()))
        self.assertEqual(set(result['activation_observations'].values()),
                         {'on_observed_after_hold_goal_write'})
        verified = [v['id'] for kind, v in events if kind == 'torque_on_verified']
        self.assertEqual(verified, list(hold.IDS))
        for i in hold.IDS[:-1]:
            completed = next(n for n, (k, v) in enumerate(events)
                             if k == 'activated_joint_feedback' and v['id'] == i)
            following = next(n for n, (k, v) in enumerate(events)
                             if k == 'write_attempt' and v['id'] == i+1)
            self.assertLess(completed, following)

    def test_mixed_activation_behaviors_supported(self):
        original = rows(); packet = Packet(original)
        def write(port, i, address, value):
            result = FakePacket.write2ByteTxRx(packet, port, i, address, value)
            if address == 30 and i % 2:
                packet.rows[i]['torque_enable'] = 1
            return result
        packet.write2ByteTxRx = write
        result = hold.prepare_hold(packet, object(), original, lambda *a, **k: None)
        self.assertEqual([i for i, address, _ in packet.writes if address == 24],
                         list(range(2, 21, 2)))
        self.assertEqual(len(result['newly_enabled_ids']), 20)

    def test_bad_feedback_after_goal_activation_stops_before_next_joint(self):
        for field, delta in (('present_ticks', 9), ('goal_ticks', 1), ('torque_limit', -1),
                             ('torque_enable', 1)):
            original = rows(); packet = GoalActivatingPacket(original)
            def event(kind, **values):
                if kind == 'after_hold_goal_write' and values['id'] == 1:
                    values['position'][field] += delta
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                hold.prepare_hold(packet, object(), original, event)
            self.assertEqual({i for i, _, _ in packet.writes}, {1})
            self.assertFalse(any(address == 24 for _, address, _ in packet.writes))

    def test_activation_before_our_goal_write_is_not_accepted(self):
        original = rows(); packet = Packet(original)
        def event(kind, **values):
            if kind == 'write_verified' and values['address'] == 32:
                packet.rows[1]['torque_enable'] = 1
        with self.assertRaisesRegex(RuntimeError, 'torque_enable changed'):
            hold.prepare_hold(packet, object(), original, event)
        self.assertEqual(packet.writes, [(1, 32, hold.DEFAULT_SPEED_CAP)])

    def test_goal_timeout_may_activate_records_risk_without_retry(self):
        original = rows(); packet = GoalActivatingPacket(original); events = []
        def write(port, i, address, value):
            result = GoalActivatingPacket.write2ByteTxRx(packet, port, i, address, value)
            return (-3001, 0) if address == 30 else result
        packet.write2ByteTxRx = write
        with self.assertRaisesRegex(RuntimeError, 'NO automatic write retry'):
            hold.prepare_hold(packet, object(), original,
                              lambda kind, **values: events.append((kind, values)))
        self.assertEqual(packet.writes, [(1, 32, hold.DEFAULT_SPEED_CAP),
                                         (1, 30, original[1]['present_ticks'])])
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertEqual(packet.rows[2]['torque_enable'], 0)
        self.assertTrue(any(k == 'hold_goal_activation_possible' for k, _ in events))
        self.assertFalse(any(k == 'torque_on_verified' for k, _ in events))

    def test_goal_activated_torque_lost_later_still_rejected(self):
        original = rows(); packet = GoalActivatingPacket(original)
        def event(kind, **values):
            if kind == 'activated_joint_feedback' and values['id'] == 20:
                packet.rows[1]['torque_enable'] = 0
        with self.assertRaisesRegex(RuntimeError, 'torque_enable changed'):
            hold.prepare_hold(packet, object(), original, event)

    def test_goal_readback_mismatch_records_possible_activation_and_stops(self):
        original = rows(); packet = GoalActivatingPacket(original); events = []
        def write(port, i, address, value):
            result = GoalActivatingPacket.write2ByteTxRx(packet, port, i, address, value)
            if address == 30:
                packet.rows[i]['goal_ticks'] += 1
            return result
        packet.write2ByteTxRx = write
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            hold.prepare_hold(packet, object(), original,
                              lambda kind, **values: events.append((kind, values)))
        self.assertEqual({i for i, _, _ in packet.writes}, {1})
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertTrue(any(k == 'hold_goal_activation_possible' for k, _ in events))
        self.assertFalse(any(k == 'torque_on_verified' for k, _ in events))

    def test_transient_moving_after_goal_activation_settles_before_next_joint(self):
        original = rows(); packet = GoalActivatingPacket(original); events = []
        def event(kind, **values):
            events.append((kind, copy.deepcopy(values)))
            if kind == 'after_hold_goal_write':
                values['position']['moving'] = 1
                packet.rows[values['id']]['moving'] = 1
            if kind == 'hold_settling_feedback':
                packet.rows[values['id']]['moving'] = 0
        result = hold.prepare_hold(packet, object(), original, event)
        self.assertEqual(result['status'], 'ALL_TORQUE_ON_VERIFIED_CURRENT_POSE')
        for i in hold.IDS:
            n = next(n for n, (k, v) in enumerate(events) if k == 'hold_settled' and v['id'] == i)
            self.assertEqual(events[n][1]['stable_samples'], 3)
            if i < 20:
                following = next(n for n, (k, v) in enumerate(events)
                                 if k == 'write_attempt' and v['id'] == i+1)
                self.assertLess(n, following)

    def test_transient_moving_after_explicit_enable_is_waited_not_rejected(self):
        original = rows(); packet = Packet(original)
        def event(kind, **values):
            if kind == 'after_torque_activation':
                values['position']['moving'] = 1
        result = hold.prepare_hold(packet, object(), original, event)
        self.assertEqual(len(result['newly_enabled_ids']), 20)
        self.assertEqual([i for i, address, _ in packet.writes if address == 24], list(hold.IDS))

    def test_persistent_moving_after_activation_stops_before_next_joint(self):
        original = rows(); packet = GoalActivatingPacket(original); events = []
        def event(kind, **values):
            events.append((kind, values))
            if kind == 'after_hold_goal_write':
                values['position']['moving'] = 1
                packet.rows[values['id']]['moving'] = 1
        with self.assertRaisesRegex(RuntimeError, 'hold settling timed out.*moving=1'):
            hold.prepare_hold(packet, object(), original, event, settle_timeout=.2)
        self.assertEqual({i for i, _, _ in packet.writes}, {1})
        self.assertFalse(any(k == 'torque_on_verified' for k, _ in events))

    def test_invalid_timeout_never_writes(self):
        for timeout in (0, -.1, 11, float('nan'), float('inf')):
            original = rows(); packet = Packet(original)
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                hold.prepare_hold(packet, object(), original, lambda *a, **k: None, settle_timeout=timeout)
            self.assertEqual(packet.writes, [])

    def test_partial_activation_failure_does_not_rollback(self):
        original = rows(); packet = Packet(original)
        def enable(port, i, address, value):
            if i == 2:
                # Model a write that applied but its acknowledgement was lost.
                packet.rows[i]['torque_enable'] = 1
                return -3001, 0
            return FakePacket.write2ByteTxRx(packet, port, i, address, value)
        packet.write1ByteTxRx = enable
        events = []
        with self.assertRaises(RuntimeError):
            hold.prepare_hold(packet, object(), original,
                              lambda kind, **values: events.append((kind, values)))
        attempted = [v['id'] for kind, v in events if kind == 'write_attempt' and v['address'] == 24]
        self.assertEqual(attempted, [1, 2])
        self.assertEqual(packet.rows[1]['torque_enable'], 1)
        self.assertEqual(packet.rows[2]['torque_enable'], 1)
        self.assertEqual(packet.rows[3]['torque_enable'], 0)
        self.assertFalse(any(address == 24 and value == 0 for _, address, value in packet.writes))

    def test_write_allowlist_and_nonzero_speed(self):
        packet = Packet(rows())
        for address, value in ((34, 1023), (6, 0), (24, 0), (24, True), (30, 4096), (32, 0)):
            with self.assertRaises(ValueError):
                hold.checked_write(packet, object(), 1, address, value, lambda *a, **k: None)
        self.assertEqual(packet.writes, [])

    def run_mock_cli(self, args, confirm='HOLD CURRENT POSE AND ENABLE TORQUE', drift=False,
                     goal_activates=False):
        original = rows(); packet = (GoalActivatingPacket if goal_activates else Packet)(original); port = Mock()
        port.openPort.return_value = True; port.setBaudRate.return_value = True
        sdk = types.SimpleNamespace(PortHandler=lambda path: port, PacketHandler=lambda protocol: packet)
        def confirmation(prompt):
            if drift:
                packet.rows[1]['present_ticks'] += 9
            return confirm
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)/'new_output'
            with patch.dict(sys.modules, {'dynamixel_sdk': sdk}), \
                 patch.object(hold, 'snapshot_configuration', return_value={'configuration_errors': []}), \
                 patch('builtins.input', side_effect=confirmation), contextlib.redirect_stdout(io.StringIO()):
                result = hold.main(args+['--output', str(destination)])
            metadata = json.loads((destination/'torque_hold_metadata.json').read_text())
        return result, packet, port, metadata

    def test_cli_inspect_read_only(self):
        result, packet, port, metadata = self.run_mock_cli(['--inspect'])
        self.assertEqual(result, 0); self.assertEqual(packet.writes, [])
        self.assertTrue(metadata['completed']); port.closePort.assert_called_once()

    def test_cli_success_logs_verified_ids(self):
        result, packet, port, metadata = self.run_mock_cli(
            ['--execute', '--robot-supported', '--exclusive-port-confirmed'])
        self.assertEqual(result, 0); self.assertTrue(metadata['completed'])
        self.assertEqual(metadata['torque_on_attempted_ids'], list(hold.IDS))
        self.assertEqual(metadata['torque_on_verified_ids'], list(hold.IDS))
        port.closePort.assert_called_once()

    def test_cli_cancel_or_prompt_drift_zero_writes(self):
        arguments = ['--execute', '--robot-supported', '--exclusive-port-confirmed']
        for confirm, drift in (('NO', False), ('HOLD CURRENT POSE AND ENABLE TORQUE', True)):
            result, packet, port, metadata = self.run_mock_cli(arguments, confirm, drift)
            self.assertEqual(result, 1); self.assertEqual(packet.writes, [])
            self.assertFalse(metadata['completed']); port.closePort.assert_called_once()

    def test_cli_goal_activation_metadata_distinguishes_explicit_writes(self):
        result, packet, port, metadata = self.run_mock_cli(
            ['--execute', '--robot-supported', '--exclusive-port-confirmed'], goal_activates=True)
        self.assertEqual(result, 0)
        self.assertEqual(metadata['torque_on_attempted_ids'], [])
        self.assertEqual(metadata['hold_goal_activation_possible_ids'], list(hold.IDS))
        self.assertEqual(metadata['torque_on_verified_ids'], list(hold.IDS))
        self.assertEqual(set(metadata['torque_activation_observations'].values()),
                         {'on_observed_after_hold_goal_write'})


class RetryTests(unittest.TestCase):
    def test_read_timeout_then_success_with_retry_event(self):
        packet = Mock(); packet.read2ByteTxRx.side_effect = [(0, -3001, 0), (2048, 0, 0)]
        packet.getTxRxResult.return_value = 'timeout'
        with patch.object(capture.time, 'sleep'):
            self.assertEqual(capture.read_value(packet, object(), 5, 36, 2), 2048)
        self.assertEqual(packet.read2ByteTxRx.call_count, 2)
        packet.read_retry_observer.assert_called_once()

    def test_corrupt_read_has_bounded_retry_and_context(self):
        packet = Mock(); packet.read2ByteTxRx.return_value = (0, -3002, 0)
        packet.getTxRxResult.return_value = 'corrupt'
        with patch.object(capture.time, 'sleep'), self.assertRaisesRegex(RuntimeError, 'ID 05 READ address=36 size=2 attempt=3/3'):
            capture.read_value(packet, object(), 5, 36, 2)
        self.assertEqual(packet.read2ByteTxRx.call_count, 3)

    def test_device_or_transmit_error_not_retried(self):
        for result, error in ((0, 4), (-1000, 0), (-1001, 0)):
            packet = Mock(); packet.read2ByteTxRx.return_value = (0, result, error)
            packet.getRxPacketError.return_value = 'device error'
            packet.getTxRxResult.return_value = 'transport error'
            with self.assertRaises(RuntimeError):
                capture.read_value(packet, object(), 5, 36, 2)
            packet.read2ByteTxRx.assert_called_once()


if __name__ == '__main__':
    unittest.main()
