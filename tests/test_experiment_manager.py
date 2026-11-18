"""Exercise real subprocesses and TCP sockets, without ROS or robot hardware."""
import importlib.util
from pathlib import Path
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


class SupervisorTests(unittest.TestCase):
    def test_reset_rejected_connection_does_not_kill_server(self):
        server = self.module.ExperimentServer(self.manager, host='127.0.0.1', port=0)
        errors = []
        def serve():
            try:
                server.serve()
            except Exception as error:
                errors.append(error)
        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 3)
        self.addCleanup(server.close)
        with socket.create_connection(server.address, timeout=2) as client:
            client.sendall(b'STATUS,none\n')
            reader = client.makefile('rb')
            self.assertEqual(reader.readline(), b'ERROR,none,unknown_request\n')
            for _ in range(5):
                rejected = socket.create_connection(server.address, timeout=2)
                rejected.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                    struct.pack('hh' if os.name == 'nt' else 'ii', 1, 0))
                rejected.close()
                time.sleep(0.03)
            self.assertEqual(errors, [], 'A rejected TCP client crashed the supervisor')
            client.sendall(b'FINISH_EXPERIMENT\n')
            self.assertEqual(reader.readline(), b'EXPERIMENT_FINISHED\n')
            reader.close()

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux /proc and setsid integration')
    def test_detached_descendant_is_killed_before_trial_cleanup(self):
        self.mode = 'detached'
        self.manager.handle('START,r1,P01,0', '127.0.0.1')
        path = self.created[0][1]
        deadline = time.monotonic() + 3
        while not (path / 'descendant').exists():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.01)
        pid = int((path / 'descendant').read_text())
        self.pump_until(lambda: self.manager.active is None)
        stat_path = Path('/proc') / str(pid) / 'stat'
        self.assertTrue(not stat_path.exists() or stat_path.read_text().split(') ')[1][0] == 'Z')
        self.assertTrue(self.manager.latest('r1').startswith('ERROR,'))

    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('experiment_manager'),
                             'The persistent TCP experiment supervisor is missing')
        import experiment_manager
        self.module = experiment_manager
        self.created = []
        self.mode = 'normal'

        def launch(request, directory):
            process = experiment_manager.SessionProcess(
                [sys.executable, str(ROOT / 'tests/fixtures/fake_session.py'), self.mode],
                directory)
            self.created.append((request, Path(directory), process))
            return process

        self.manager = experiment_manager.ExperimentManager(launch, startup_timeout=2,
                                                            shutdown_timeout=0.15)
        self.addCleanup(self.manager.close)

    def pump_until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.manager.tick()
            if time.monotonic() >= deadline:
                self.fail('Supervisor did not reach expected state')
            time.sleep(0.01)

    def start_ready(self, request='r1', participant='P01', session='0'):
        self.assertEqual(self.manager.handle('START,%s,%s,%s' %
                                            (request, participant, session), '127.0.0.1'),
                         ['STARTING,%s,%s,%s' % (request, participant, session)])
        self.pump_until(lambda: self.manager.latest(request).startswith('READY,'))

    def release(self):
        (self.created[-1][1] / 'release').touch()
        self.pump_until(lambda: self.manager.active is None)

    def test_two_trials_only_report_done_after_completed_child_exit(self):
        for request, session in [('r1', '0'), ('r2', '1')]:
            self.start_ready(request, session=session)
            self.assertTrue((self.created[-1][1] / 'accept_commands').exists())
            self.assertIsNone(self.created[-1][2].poll())
            self.release()
            self.assertEqual(self.manager.latest(request), 'DONE,%s,P01,%s' %
                             (request, session))
        self.assertEqual(len(self.created), 2)

    def test_ready_waits_for_both_bridge_and_controller(self):
        self.manager.handle('START,r1,P01,0', '127.0.0.1')
        self.pump_until(lambda: (self.created[0][1] / 'controller_ready').exists())
        self.assertEqual(self.manager.latest('r1'), 'STARTING,r1,P01,0')
        self.assertFalse((self.created[0][1] / 'accept_commands').exists())
        self.pump_until(lambda: self.manager.latest('r1').startswith('READY,'))
        self.release()

    def test_duplicate_replay_conflict_busy_and_peer_binding(self):
        self.start_ready()
        self.assertEqual(self.manager.handle('START,r1,P01,0', '127.0.0.1'),
                         ['READY,r1,P01,0'])
        self.assertEqual(len(self.created), 1)
        self.assertEqual(self.manager.handle('START,r1,P02,0', '127.0.0.1'),
                         ['ERROR,r1,request_conflict'])
        self.assertEqual(self.manager.handle('START,r2,P01,1', '127.0.0.1'), ['BUSY,r2'])
        self.assertEqual(self.manager.handle('START,r3,P01,1', '127.0.0.2'),
                         ['ERROR,r3,wrong_peer'])
        self.release()
        self.assertEqual(self.manager.handle('START,r1,P01,0', '127.0.0.1'),
                         ['DONE,r1,P01,0'])

    def test_invalid_ids_and_protocol_do_not_launch(self):
        for line in ['START,r1,p01,0', 'START,r1,P01,-1', 'START,r1,P01,1.5',
                     'START,../bad,P01,0', 'START,r1,P01,01', 'START,r1,P01',
                     'START,r1,P01,0,extra', 'BOGUS']:
            self.assertTrue(self.manager.handle(line, '127.0.0.1')[0].startswith('ERROR,'))
        self.assertEqual(self.created, [])

    def test_exit_without_completion_or_with_nonzero_status_is_error(self):
        for mode in ('fail', 'incomplete', 'bad_exit'):
            self.mode = mode
            request = mode
            self.manager.handle('START,%s,P01,0' % request, '127.0.0.1')
            if mode != 'fail':
                self.pump_until(lambda: self.manager.latest(request).startswith('READY,'))
                (self.created[-1][1] / 'release').touch()
            self.pump_until(lambda: self.manager.active is None)
            self.assertTrue(self.manager.latest(request).startswith('ERROR,'))

    def test_startup_timeout_reaps_child_and_allows_another_trial(self):
        self.mode = 'hang'
        self.manager.startup_timeout = 0.1
        self.manager.handle('START,r1,P01,0', '127.0.0.1')
        self.pump_until(lambda: self.manager.active is None)
        self.assertEqual(self.manager.latest('r1'), 'ERROR,r1,startup_timeout')
        self.assertIsNotNone(self.created[0][2].poll())
        self.mode = 'normal'
        self.manager.startup_timeout = 2
        self.start_ready('r2')
        self.release()

    def test_finish_waits_for_child_then_emits_done_before_finished(self):
        self.start_ready()
        self.manager.drain_events()
        self.assertEqual(self.manager.handle('FINISH_EXPERIMENT', '127.0.0.1'),
                         ['FINISHING'])
        self.assertFalse(self.manager.finished)
        self.assertEqual(self.manager.handle('START,r2,P01,1', '127.0.0.1'),
                         ['ERROR,r2,experiment_finishing'])
        self.release()
        self.assertTrue(self.manager.finished)
        self.assertEqual(self.manager.drain_events(), ['DONE,r1,P01,0',
                                                       'EXPERIMENT_FINISHED'])

    def test_tcp_framing_and_reconnect_replay(self):
        server = self.module.ExperimentServer(self.manager, host='127.0.0.1', port=0)
        thread = threading.Thread(target=server.serve, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 3)
        self.addCleanup(server.close)
        client = socket.create_connection(server.address, timeout=2)
        self.addCleanup(client.close)
        client.sendall(b'START,r1,P')
        time.sleep(0.03)
        client.sendall(b'01,0\nSTATUS,r1\n')
        reader = client.makefile('rb')
        self.assertEqual(reader.readline(), b'STARTING,r1,P01,0\n')
        self.assertEqual(reader.readline(), b'STARTING,r1,P01,0\n')
        self.assertEqual(reader.readline(), b'READY,r1,P01,0\n')
        reader.close()
        client.close()
        time.sleep(0.1)
        with socket.create_connection(server.address, timeout=2) as second:
            second.sendall(b'STATUS,r1\n')
            read = second.makefile('rb')
            self.assertEqual(read.readline(), b'READY,r1,P01,0\n')
            (self.created[-1][1] / 'release').touch()
            self.assertEqual(read.readline(), b'DONE,r1,P01,0\n')
            second.sendall(b'FINISH_EXPERIMENT\n')
            self.assertEqual(read.readline(), b'EXPERIMENT_FINISHED\n')
            read.close()
        thread.join(3)
        self.assertFalse(thread.is_alive())


if __name__ == '__main__':
    unittest.main()
