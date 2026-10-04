"""Offline transport tests: synthetic register bytes, no SDK/serial connection."""
import copy
import unittest
from unittest.mock import Mock, patch

from test_centerized_reference import FakePacket, Clock, baseline, capture, reference


class BlockReadTests(unittest.TestCase):
    def test_block_decode_and_transaction_count(self):
        packet = FakePacket()
        row = capture.capture_sample(packet, object(), 8, extended=True)
        self.assertEqual(row['goal_ticks'], baseline()[8]['goal_ticks'])
        self.assertEqual(row['model_number'], 310)
        self.assertEqual(packet.block_reads, [(8, 2, 45)])
        self.assertEqual(packet.reads, [(8, 0), (8, 70)])

    def test_unsigned_little_endian_and_bounds(self):
        self.assertEqual(capture.decode_register([0x34, 0x12], 30, 30, 2), 0x1234)
        self.assertEqual(capture.decode_register([255, 255], 30, 30, 2), 65535)
        for addr, size in ((29, 1), (31, 2), (30, 3)):
            with self.assertRaises(ValueError):
                capture.decode_register([0, 0], 30, addr, size)

    def test_short_or_invalid_block_never_used_as_positions(self):
        for data in ([1], [0, 256], [0, -1], [0, float('nan')]):
            packet = Mock(); packet.readTxRx.return_value = (data, 0, 0)
            with self.subTest(data=data), self.assertRaisesRegex(RuntimeError, 'invalid length/bytes'):
                capture.read_block(packet, object(), 1, 30, 2)
            packet.readTxRx.assert_called_once()

    def test_timeout_and_corrupt_read_bounded_retry(self):
        packet = Mock()
        packet.getTxRxResult.return_value = 'timeout/corrupt'
        packet.readTxRx.side_effect = [([], -3001, 0), ([], -3002, 0), ([1, 2], 0, 0)]
        with patch.object(capture.time, 'sleep'):
            self.assertEqual(capture.read_block(packet, object(), 8, 30, 2), [1, 2])
        self.assertEqual(packet.readTxRx.call_count, 3)
        self.assertEqual(packet.read_retry_observer.call_count, 2)

    def test_permanent_timeout_has_register_context(self):
        packet = Mock(); packet.readTxRx.return_value = ([], -3001, 0)
        packet.getTxRxResult.return_value = 'no status packet'
        with patch.object(capture.time, 'sleep'), self.assertRaisesRegex(
                RuntimeError, 'ID 16 BLOCK READ address=0 length=47 attempt=3/3'):
            capture.read_block(packet, object(), 16, 0, 47)
        self.assertEqual(packet.readTxRx.call_count, 3)

    def test_device_or_transmit_error_never_retried(self):
        for result, error in ((0, 4), (-1001, 0)):
            packet = Mock(); packet.readTxRx.return_value = ([], result, error)
            packet.getTxRxResult.return_value = 'transmit error'
            packet.getRxPacketError.return_value = 'device error'
            with self.assertRaises(RuntimeError):
                capture.read_block(packet, object(), 1, 0, 47)
            packet.readTxRx.assert_called_once()


class SyncTransportTests(unittest.TestCase):
    def run_ramp(self, packet, initial=None):
        events = []; clock = Clock()
        result = reference.execute_small_correction(packet, object(), initial or baseline(),
            lambda kind, **v: events.append((kind, copy.deepcopy(v))), sleep=clock.sleep, clock=clock.now)
        return result, events

    def test_sync_packets_replace_per_joint_ack_writes(self):
        packet = FakePacket(); result, events = self.run_ramp(packet)
        self.assertEqual([a for a, _, _, _ in packet.sync_packets], [32, 30, 30, 30])
        first = packet.sync_packets[0]
        self.assertEqual(first[1], 2); self.assertEqual(first[3], 60)
        self.assertEqual(first[2][:3], [1, 20, 0])
        self.assertEqual(result['status'], 'REFERENCE_GOALS_REACHED_FEEDBACK_NEAR_TARGET')
        tx_indices = [n for n, (k, v) in enumerate(events) if k == 'sync_write_transmitted' and v['address'] == 30]
        verified = [n for n, (k, v) in enumerate(events) if k == 'sync_write_verified']
        self.assertEqual(len(verified), 3)
        for n in range(2):
            self.assertLess(tx_indices[n], verified[n]); self.assertLess(verified[n], tx_indices[n+1])
        self.assertLess(tx_indices[-1], verified[-1])

    def test_tx_success_without_goal_application_stops_before_second_frame(self):
        packet = FakePacket(); original = packet.syncWriteTxOnly
        def drop_goal(port, address, size, params, length):
            if address == 30:
                packet.sync_packets.append((address, size, list(params), length))
                return 0  # Broadcast transmit success; one or all servos can ignore it.
            return original(port, address, size, params, length)
        packet.syncWriteTxOnly = drop_goal
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            self.run_ramp(packet)
        self.assertEqual([a for a, _, _, _ in packet.sync_packets], [32, 30])

    def test_unverified_speed_caps_prevent_any_goal_frame(self):
        packet = FakePacket()
        packet.syncWriteTxOnly = Mock(return_value=0)
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            self.run_ramp(packet)
        packet.syncWriteTxOnly.assert_called_once()
        self.assertEqual(packet.syncWriteTxOnly.call_args.args[1], 32)

    def test_partial_tx_failure_no_retry_and_no_pose_success(self):
        packet = FakePacket(); original = packet.syncWriteTxOnly
        def fail_goal(port, address, size, params, length):
            if address == 30:
                packet.sync_packets.append((address, size, list(params), length))
                packet.write2ByteTxRx(port, params[0], 30, params[1] | (params[2] << 8))
                return -1001
            return original(port, address, size, params, length)
        packet.syncWriteTxOnly = fail_goal
        with self.assertRaisesRegex(RuntimeError, 'NO automatic write retry'):
            self.run_ramp(packet)
        self.assertEqual([a for a, _, _, _ in packet.sync_packets], [32, 30])
        self.assertNotEqual(packet.rows[1]['goal_ticks'], baseline()[1]['goal_ticks'])
        self.assertEqual(packet.rows[2]['goal_ticks'], baseline()[2]['goal_ticks'])

    def test_runtime_reads_32_transactions_and_records_bad_feedback_before_refusal(self):
        packet = FakePacket()
        for row in packet.rows.values(): row['moving_speed_raw'] = 20
        packet.rows[19]['temperature_c'] = 55
        events = []
        with self.assertRaisesRegex(RuntimeError, 'observed=55 C threshold=55 C'):
            reference.check_runtime(packet, object(), baseline(),
                {i: r['goal_ticks'] for i, r in baseline().items()}, lambda k, **v: events.append((k, v)))
        self.assertEqual(events[-1][1]['id'], 19)
        self.assertEqual(events[-1][1]['position']['temperature_c'], 55)
        packet.rows[19]['temperature_c'] = 35; packet.block_reads.clear(); packet.reads.clear()
        reference.check_runtime(packet, object(), baseline(),
                                {i: r['goal_ticks'] for i, r in baseline().items()})
        self.assertEqual(len(packet.block_reads), 20)
        self.assertEqual(len(packet.reads), 12)

    def test_frame_overrun_is_logged_not_claimed_as_nominal_rate(self):
        packet = FakePacket(); clock = Clock(); events = []
        read = packet.readTxRx
        def slow_read(*args):
            clock.t += .02
            return read(*args)
        packet.readTxRx = slow_read
        reference.execute_small_correction(packet, object(), baseline(),
            lambda k, **v: events.append((k, v)), sleep=clock.sleep, clock=clock.now)
        timing = [v for k, v in events if k == 'frame_timing']
        self.assertTrue(all(v['overrun'] for v in timing))
        self.assertTrue(all(v['elapsed_s'] >= .39 for v in timing))

    def test_changed_model_stops_before_mx_specific_read(self):
        packet = FakePacket()
        for row in packet.rows.values(): row['moving_speed_raw'] = 20
        packet.rows[8]['model_number'] = 999
        with self.assertRaisesRegex(RuntimeError, 'model/ID changed'):
            reference.check_runtime(packet, object(), baseline(),
                                    {i: r['goal_ticks'] for i, r in baseline().items()})
        self.assertNotIn((8, 70), packet.reads)

    def test_missing_speed_id_or_invalid_goal_ids_rejected_before_tx(self):
        packet = FakePacket()
        for address, values in ((32, {1: 20}), (30, {21: 2048}), (30, {True: 2048}), (30, {})):
            with self.assertRaises(ValueError):
                reference.sync_write(packet, object(), address, values, lambda *a, **k: None)
        self.assertEqual(packet.sync_packets, [])


if __name__ == '__main__':
    unittest.main()
