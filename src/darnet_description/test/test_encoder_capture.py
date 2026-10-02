"""Offline tests: no ROS, serial port, or physical motor required."""

import ast
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'darnet_description'
spec = importlib.util.spec_from_file_location(
    'encoder_capture', SOURCE / 'CaptureEncoderReference.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class ReadOnlyPacket:
    """A fake SDK object that deliberately has no write methods."""

    def __init__(self, model=29):
        self.model = model
        self.addresses = []

    def read2ByteTxRx(self, port, servo_id, address):
        self.addresses.append(address)
        return (self.model if address == 0 else 2048), 0, 0

    def read1ByteTxRx(self, port, servo_id, address):
        self.addresses.append(address)
        return 1, 0, 0


class CaptureTests(unittest.TestCase):
    def test_capture_only_reads(self):
        packet = ReadOnlyPacket()
        row = capture.capture_sample(packet, object(), 1)
        self.assertEqual(row['present_ticks'], 2048)
        self.assertEqual(row['torque_enable'], 1)
        self.assertEqual(packet.addresses, [0, 6, 8, 22, 24, 30, 36, 42, 43])

    def test_unknown_model_stops_before_legacy_registers(self):
        packet = ReadOnlyPacket(model=999)
        with self.assertRaisesRegex(RuntimeError, 'Unsupported model'):
            capture.capture_sample(packet, object(), 1)
        self.assertEqual(packet.addresses, [0])

    def test_transport_and_device_errors_not_used_as_ticks(self):
        for result, error in ((-1, 0), (0, 4)):
            packet = Mock()
            packet.read2ByteTxRx.return_value = (0, result, error)
            packet.getTxRxResult.return_value = 'transport error'
            packet.getRxPacketError.return_value = 'device error'
            with self.assertRaises(RuntimeError):
                capture.read_value(packet, object(), 1, 36, 2)

    def test_reader_source_has_no_sdk_writes(self):
        tree = ast.parse((SOURCE / 'CaptureEncoderReference.py').read_text())
        attributes = [node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)]
        self.assertFalse(any(name.startswith('write') and 'Tx' in name for name in attributes))


class BridgeShutdownTests(unittest.TestCase):
    def make_bridge(self, keep):
        # Compile only the actual shutdown method: no ROS imports/startup writes.
        tree = ast.parse((SOURCE / 'ComsROS2U2D2.py').read_text())
        source_class = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in source_class.body
                      if isinstance(n, ast.FunctionDef) and n.name == 'destroy_node')

        class Base:
            def destroy_node(self):
                self.destroyed = True

        isolated = ast.ClassDef(name='IsolatedBridge', bases=[ast.Name(id='Base', ctx=ast.Load())],
                                keywords=[], body=[method], decorator_list=[])
        namespace = {'Base': Base, 'DXL_IDS': {'first': 1, 'second': 2},
                     'ADDR_TORQUE_ENABLE': 24}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[isolated], type_ignores=[])),
                     '<shutdown-test>', 'exec'), namespace)
        bridge = namespace['IsolatedBridge']()
        bridge.keep_torque_on_exit = keep
        bridge.port = Mock()
        bridge.packet = Mock()
        bridge.get_logger = Mock(return_value=Mock())
        return bridge

    def test_keep_torque_performs_no_write(self):
        bridge = self.make_bridge(True)
        bridge.destroy_node()
        bridge.packet.write1ByteTxRx.assert_not_called()
        bridge.port.closePort.assert_called_once()
        self.assertTrue(bridge.destroyed)

    def test_default_shutdown_disables_torque(self):
        bridge = self.make_bridge(False)
        bridge.destroy_node()
        self.assertEqual(bridge.packet.write1ByteTxRx.call_count, 2)
        bridge.packet.write1ByteTxRx.assert_any_call(bridge.port, 1, 24, 0)
        bridge.port.closePort.assert_called_once()

    def test_port_closed_on_shutdown_write_exception(self):
        bridge = self.make_bridge(False)
        bridge.packet.write1ByteTxRx.side_effect = RuntimeError('serial failure')
        with self.assertRaises(RuntimeError):
            bridge.destroy_node()
        bridge.port.closePort.assert_called_once()
        self.assertTrue(bridge.destroyed)


if __name__ == '__main__':
    unittest.main()
