"""Validate the Windows-compatible client against the real loopback server."""
import importlib.util
import math
from pathlib import Path
import socket
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import fmp_udp_sender as sender_module
import test_experiment_manager as supervisor_tests


class ClientTests(unittest.TestCase):
    def test_status_command_waits_for_a_complete_response(self):
        self.assertTrue(hasattr(sender_module, 'managed_experiment'))
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            def reply():
                connection, _ = listener.accept()
                with connection:
                    self.assertEqual(connection.recv(1024), b'STATUS,r1\n')
                    connection.sendall(b'REA')
                    time.sleep(0.08)
                    try:
                        connection.sendall(b'DY,r1,P01,0\n')
                    except OSError:
                        pass
            thread = threading.Thread(target=reply)
            thread.start()
            args = types.SimpleNamespace(host='127.0.0.1', tcp_port=listener.getsockname()[1],
                                         port=5005, status='r1')
            with patch('builtins.print') as output:
                result = sender_module.managed_experiment(args, lambda: None)
            thread.join(1)
            self.assertEqual(result, 0)
            self.assertIn(('READY,r1,P01,0',), [call.args for call in output.call_args_list])

    def test_finish_waits_for_ack_even_if_active_trial_fails(self):
        self.assertTrue(hasattr(sender_module, 'wait_experiment_finished'))
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            client = sender_module.ExperimentClient('127.0.0.1', listener.getsockname()[1])
            self.addCleanup(client.close)
            connection, _ = listener.accept()
            self.addCleanup(connection.close)
            connection.sendall(b'FINISHING\nERROR,r1,child_exit_7\n')
            def finish_later():
                time.sleep(0.08)
                connection.sendall(b'EXPERIMENT_FINISHED\n')
            thread = threading.Thread(target=finish_later)
            thread.start()
            self.addCleanup(thread.join, 1)
            with patch('builtins.print'):
                result = sender_module.wait_experiment_finished(client)
            self.assertTrue(client.experiment_finished)
            self.assertFalse(result, 'Failed trial should not become successful at experiment finish')

    def test_existing_udp_sender_keeps_ascii_payload(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(1)
            sender = sender_module.CommandSender('127.0.0.1', receiver.getsockname()[1])
            self.addCleanup(sender._socket.close)
            sender.send(-1)
            fields = receiver.recv(1024).decode('ascii').rstrip('\n').split(',')
            self.assertEqual(fields[:2], ['1', '-1'])
            self.assertTrue(math.isfinite(float(fields[2])))

    def test_invalid_pattern_is_rejected_before_any_trial_starts(self):
        for pattern in ['2:1', '1:-1', '1:nan', '1:inf', '', '1:0']:
            with self.subTest(pattern=pattern):
                with self.assertRaises(ValueError):
                    sender_module.parse_pattern(pattern)

    def test_client_buffers_partial_lines_and_rejects_invalid_start_locally(self):
        self.assertTrue(hasattr(sender_module, 'ExperimentClient'), 'TCP client missing')
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            client = sender_module.ExperimentClient('127.0.0.1', listener.getsockname()[1])
            self.addCleanup(client.close)
            connection, _ = listener.accept()
            self.addCleanup(connection.close)
            with self.assertRaises(ValueError):
                client.start('P01,evil', '0')
            request = client.start('P01', '0', request_id='test')
            self.assertEqual(request, 'test')
            self.assertEqual(connection.recv(1024), b'START,test,P01,0\n')
            connection.sendall(b'STARTING,test,P01,0\nREA')
            self.assertEqual(client.poll(1), ['STARTING,test,P01,0'])
            connection.sendall(b'DY,test,P01,0\nDONE,test,P01,0\n')
            self.assertEqual(client.poll(1), ['READY,test,P01,0', 'DONE,test,P01,0'])
            client.status('test')
            self.assertEqual(connection.recv(1024), b'STATUS,test\n')
            client.finish()
            self.assertEqual(connection.recv(1024), b'FINISH_EXPERIMENT\n')


class ManagedFlowTests(unittest.TestCase):
    setUp = supervisor_tests.SupervisorTests.setUp
    def test_windows_client_waits_for_ready_sends_udp_then_observes_done(self):
        self.assertTrue(hasattr(sender_module, 'run_managed_trial'), 'Managed sender missing')
        server = self.module.ExperimentServer(self.manager, '127.0.0.1', 0)
        server_thread = threading.Thread(target=server.serve, daemon=True)
        server_thread.start()
        self.addCleanup(server_thread.join, 3)
        self.addCleanup(server.close)
        client = sender_module.ExperimentClient(*server.address)
        self.addCleanup(client.close)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(2)
            sender = sender_module.CommandSender('127.0.0.1', receiver.getsockname()[1])
            self.addCleanup(sender._socket.close)
            request = client.start('P01', '0', request_id='trial')
            statuses = []
            failures = []
            result = []
            def run():
                try:
                    result.append(sender_module.run_managed_trial(
                        client, sender, request, lambda: 1, on_status=statuses.append))
                except Exception as error:
                    failures.append(error)
            trial_thread = threading.Thread(target=run)
            trial_thread.start()
            packet = receiver.recv(1024)
            self.assertTrue((self.created[0][1] / 'accept_commands').exists())
            self.assertTrue(any(line.startswith('READY,') for line in statuses))
            self.assertEqual(packet.decode('ascii').split(',')[1], '1')
            (self.created[0][1] / 'release').touch()
            trial_thread.join(3)
            self.assertFalse(trial_thread.is_alive())
            self.assertEqual(failures, [])
            self.assertEqual(result, ['DONE'])
            client.finish()
            self.assertIn('EXPERIMENT_FINISHED', client.poll(2))
