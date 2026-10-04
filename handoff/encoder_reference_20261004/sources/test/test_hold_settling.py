"""Offline settling state-machine tests with a synthetic monotonic clock."""
import copy
import unittest
from unittest.mock import patch

from test_centerized_reference import Clock
from test_torque_hold import Packet, rows, hold


class SettlingTests(unittest.TestCase):
    def fixture(self):
        initial = rows(off_ids=[])
        packet = Packet(initial)
        return packet, initial[1], Clock(), []

    def wait(self, packet, expected, clock, events, initial=None, timeout=.5):
        return hold.wait_for_hold_settle(packet, object(), 1, initial or dict(expected),
            expected, expected['present_ticks'],
            lambda k, **v: events.append((k, copy.deepcopy(v))),
            timeout=timeout, sleep=clock.sleep, clock=clock.now)

    def test_requires_three_consecutive_samples_and_resets_streak(self):
        packet, expected, clock, events = self.fixture()
        sequence = [0, 1, 0, 0, 0]
        snapshots = [{**expected, 'moving': moving} for moving in sequence[1:]]
        with patch.object(hold, 'capture_sample', side_effect=snapshots):
            result = self.wait(packet, expected, clock, events)
        feedback = [v for k, v in events if k == 'hold_settling_feedback']
        self.assertEqual([v['position']['moving'] for v in feedback], sequence)
        self.assertEqual(result['moving'], 0)
        self.assertEqual(events[-1][0], 'hold_settled')
        self.assertEqual(events[-1][1]['stable_samples'], 3)
        self.assertEqual(packet.writes, [])

    def test_nonzero_speed_is_waited_even_when_flag_is_zero(self):
        packet, expected, clock, events = self.fixture()
        initial = {**expected, 'present_speed_raw': 6}
        self.wait(packet, expected, clock, events, initial=initial)
        self.assertEqual(len([v for k, v in events if k == 'hold_settling_feedback']), 4)
        self.assertEqual(packet.writes, [])

    def test_persistent_speed_times_out_with_details(self):
        packet, expected, clock, events = self.fixture()
        packet.rows[1]['present_speed_raw'] = 1030  # Direction bit + magnitude 6.
        with self.assertRaisesRegex(RuntimeError, 'timed out.*speed_raw=1030'):
            self.wait(packet, expected, clock, events, initial=dict(packet.rows[1]), timeout=.2)
        self.assertFalse(any(k == 'hold_settled' for k, _ in events))
        self.assertEqual(packet.writes, [])

    def test_every_nonstationary_guard_remains_strict_during_wait(self):
        cases = (('present_ticks', 9, 'pose drift'), ('goal_ticks', 1, 'goal_ticks changed'),
                 ('torque_limit', -1, 'torque_limit changed'),
                 ('torque_enable', -1, 'torque_enable changed'),
                 ('temperature_c', 25, 'temperature guard'),
                 ('voltage_raw', -100, 'voltage outside'),
                 ('cw_limit', 1, 'cw_limit changed'),
                 ('registered_instruction', 1, 'pending registered'),
                 ('moving', 2, 'invalid movement'),
                 ('present_speed_raw', 2048, 'invalid movement'))
        for field, delta, message in cases:
            packet, expected, clock, events = self.fixture()
            initial = {**expected, 'moving': 1}
            bad = dict(initial); bad[field] = expected[field]+delta
            with self.subTest(field=field), patch.object(hold, 'capture_sample', return_value=bad):
                with self.assertRaisesRegex(RuntimeError, message):
                    self.wait(packet, expected, clock, events, initial=initial)
            self.assertEqual(len([v for k, v in events if k == 'hold_settling_feedback']), 2)
            self.assertFalse(any(k == 'hold_settled' for k, _ in events))
            self.assertEqual(packet.writes, [])

    def test_read_latency_counts_toward_deadline_not_a_false_pass(self):
        packet, expected, clock, events = self.fixture()
        def slow_read(*args, **kwargs):
            clock.t += .15
            return dict(expected)
        with patch.object(hold, 'capture_sample', side_effect=slow_read):
            with self.assertRaisesRegex(RuntimeError, 'hold settling timed out'):
                self.wait(packet, expected, clock, events, timeout=.3)
        # Three stationary observations, but after the allowed elapsed time.
        self.assertEqual(len([v for k, v in events if k == 'hold_settling_feedback']), 3)
        self.assertFalse(any(k == 'hold_settled' for k, _ in events))

    def test_read_failure_aborts_no_write_or_success(self):
        packet, expected, clock, events = self.fixture()
        with patch.object(hold, 'capture_sample', side_effect=RuntimeError('no status packet')):
            with self.assertRaisesRegex(RuntimeError, 'no status packet'):
                self.wait(packet, expected, clock, events, initial={**expected, 'moving': 1})
        self.assertEqual(packet.writes, [])
        self.assertFalse(any(k == 'hold_settled' for k, _ in events))

    def test_unrelated_preflight_still_rejects_motion(self):
        initial = rows(); initial[1]['moving'] = 1
        self.assertTrue(any('servo not stationary' in e for e in hold.preflight(initial)))

    def test_invalid_cli_timeouts_rejected_before_opening_port(self):
        import contextlib
        import io
        import CenterizedReference as reference
        for module in (hold, reference):
            for value in ('0', '-1', '11', 'nan', 'inf'):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                    module.main(['--hold-settle-timeout', value])
                self.assertEqual(exc.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
