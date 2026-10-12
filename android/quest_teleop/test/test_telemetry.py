#!/usr/bin/env python3
"""Exercise the actual C++ adapter/USB sender against a robot-free TCP peer."""
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


def read_exact(conn, size):
    result = b''
    while len(result) < size:
        chunk = conn.recv(size - len(result))
        if not chunk:
            raise EOFError
        result += chunk
    return result


class TelemetryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.binary = str(Path(cls.temp.name) / 'telemetry_fixture')
        # OS chooses an isolated test port; the deployment port stays unchanged.
        cls.server = socket.socket()
        cls.server.bind(('127.0.0.1', 0))
        cls.server.listen(1)
        cls.server.settimeout(5)
        port = cls.server.getsockname()[1]
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread',
                        f'-DQUEST_BRIDGE_PORT={port}', '-I', str(ROOT / 'test/stubs'),
                        '-I', str(ROOT / 'app/src/main/cpp'),
                        str(ROOT / 'test/telemetry_fixture.cpp'), '-o', cls.binary], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.server.close()
        cls.temp.cleanup()

    def test_packets_clock_gate_and_local_deadman(self):
        proc = subprocess.Popen([self.binary])
        packets = []
        syncs = 0
        try:
            conn, _ = self.server.accept()
            with conn:
                conn.settimeout(5)
                while True:
                    try:
                        length, = struct.unpack('!I', read_exact(conn, 4))
                        self.assertTrue(0 < length <= 4096)
                        packet = json.loads(read_exact(conn, length))
                    except EOFError:
                        break
                    if packet['type'] == 'clock_sync_request':
                        syncs += 1
                        # Delay before p2 models path delay, not server compute time.
                        if syncs == 1:
                            time.sleep(0.03)
                        p2 = time.monotonic_ns()
                        reply = dict(version=1, type='clock_sync_reply',
                                     sync_id=packet['sync_id'],
                                     quest_send_time_ns=packet['quest_send_time_ns'],
                                     pc_receive_time_ns=str(p2),
                                     pc_send_time_ns=str(time.monotonic_ns()))
                        body = json.dumps(reply, separators=(',', ':')).encode()
                        conn.sendall(struct.pack('!I', len(body)) + body)
                    else:
                        packets.append(packet)
            self.assertEqual(proc.wait(timeout=5), 0)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        self.assertGreaterEqual(syncs, 2)
        self.assertGreater(len(packets), 100)
        self.assertTrue(any(not p['clock_sync_valid'] and
                            int(p['clock_uncertainty_ns']) >= 10_000_000 for p in packets))
        self.assertTrue(any(p['clock_sync_valid'] and
                            int(p['clock_uncertainty_ns']) <= 10_000_000 for p in packets))
        sequences = [int(p['sequence']) for p in packets]
        self.assertEqual(sequences, sorted(set(sequences)))
        self.assertEqual(len({p['session_id'] for p in packets}), 1)
        self.assertEqual({p['reference_space_generation'] for p in packets}, {'0', '1'})
        per_hand_time = {'left': 0, 'right': 0}
        for p in packets:
            hand = p['controller_id']
            self.assertEqual(p['version'], 1)
            self.assertEqual(p['reference_space'], 'LOCAL')
            self.assertTrue(p['session_focused'])
            acquisition = int(p['acquisition_time_ns'])
            self.assertGreater(acquisition, per_hand_time[hand])
            per_hand_time[hand] = acquisition
            self.assertEqual(int(p['openxr_location_time_ns']) - acquisition, 10_000_000)
            self.assertTrue(p['pose']['pose_action_active'])
            self.assertEqual(p['pose']['orientation_xyzw'], [0, 0, 0, 1])
            inputs, twist = p['inputs'], p['native_twist']
            if hand == 'left':
                self.assertEqual(p['pose']['position'][0], -0.25)
                self.assertTrue(inputs['button_x'] and inputs['button_x_active'])
                self.assertNotIn('button_a', inputs)
                self.assertFalse(inputs['trigger_active'])
                self.assertFalse(twist['linear_valid'])
                self.assertTrue(twist['angular_valid'])
            else:
                self.assertEqual(p['pose']['position'][0], 0.25)
                self.assertTrue(inputs['button_a'] and inputs['button_a_active'])
                self.assertFalse(inputs['button_b_active'])
                self.assertNotIn('button_x', inputs)
                self.assertNotIn('menu_button', inputs)
                self.assertTrue(twist['linear_valid'])
                self.assertFalse(twist['angular_valid'])


if __name__ == '__main__':
    unittest.main()
