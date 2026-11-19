"""ROS boundaries are stubbed; files, UDP transport and session main are real."""
import importlib.util
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load_node(name, rospy):
    message = types.SimpleNamespace(STATE_WAITING=0, STATE_ROTATING=1, STATE_DONE=2)
    class Command:
        ANTI_CLOCKWISE, NONE, CLOCKWISE = -1, 0, 1
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
    stubs = {'rospy': rospy,
             'actionlib': types.SimpleNamespace(SimpleActionClient=object),
             'control_msgs.msg': types.SimpleNamespace(FollowJointTrajectoryAction=object,
                 FollowJointTrajectoryGoal=object, FollowJointTrajectoryResult=object),
             'controller_manager_msgs.srv': types.SimpleNamespace(ListControllers=object,
                 SwitchController=object, SwitchControllerRequest=object),
             'sensor_msgs.msg': types.SimpleNamespace(JointState=object),
             'trajectory_msgs.msg': types.SimpleNamespace(JointTrajectoryPoint=object),
             'franka_mind_pour.msg': types.SimpleNamespace(PourState=message, PourCommand=Command)}
    spec = importlib.util.spec_from_file_location(name, ROOT / 'franka_mind_pour/scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class LifecycleTests(unittest.TestCase):
    def test_move_stage_marks_success_only_after_reaching_start(self):
        for success in (True, False):
            with self.subTest(success=success), tempfile.TemporaryDirectory() as directory:
                params = {'~joint_pose': {'joint': 1}, '~participant': 'debug', '~session': '0',
                          '~stage': 'move', '~results_file': str(Path(directory) / 'results.csv')}
                rospy = types.SimpleNamespace(init_node=lambda *a: None,
                    get_param=lambda name, default=None: params.get(name, default),
                    loginfo=lambda *a: None, logerr=lambda *a: None)
                module = load_node('session_manager', rospy)
                def move(pose, max_dq):
                    self.assertEqual((pose, max_dq), ({'joint': 1}, 0.2))
                    self.assertEqual(list(Path(directory).iterdir()), [])
                    return success
                with patch.dict(os.environ, {'FMP_STATUS_DIR': directory}), \
                        patch.object(module, 'wait_for_controllers', return_value=True), \
                        patch.object(module, 'move_to_start', side_effect=move) as moved:
                    self.assertEqual(module.main(), 0 if success else 1)
                moved.assert_called_once()
                self.assertEqual([path.name for path in Path(directory).iterdir()],
                                 ['completed'] if success else [])

    def test_bridge_discards_pre_ready_packets_then_preserves_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            stopped = threading.Event()
            published = []
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rospy = types.SimpleNamespace(init_node=lambda *a: None,
                get_param=lambda name, default: port if name == '~port' else default,
                Publisher=lambda *a, **k: types.SimpleNamespace(publish=published.append),
                is_shutdown=stopped.is_set, Time=types.SimpleNamespace(now=lambda: 123),
                loginfo=lambda *a: None, logwarn=lambda *a: None,
                logwarn_throttle=lambda *a: None)
            module = load_node('udp_bridge', rospy)
            failures = []
            def run():
                try:
                    module.main()
                except Exception as error:
                    failures.append(error)
            with patch.dict(os.environ, {'FMP_STATUS_DIR': directory}):
                thread = threading.Thread(target=run)
                thread.start()
                try:
                    deadline = time.monotonic() + 1
                    while not (Path(directory) / 'bridge_ready').exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue((Path(directory) / 'bridge_ready').exists(),
                                    'Bridge never announces its bound UDP socket')
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        sender.sendto(b'1,1,100.0\n', ('127.0.0.1', port))
                        time.sleep(0.08)
                        self.assertEqual(published, [], 'Motion leaked before READY')
                        (Path(directory) / 'accept_commands').touch()
                        sender.sendto(b'2,-1,101.0\n', ('127.0.0.1', port))
                        deadline = time.monotonic() + 1
                        while not published and time.monotonic() < deadline:
                            time.sleep(0.01)
                    self.assertEqual([(msg.seq, msg.command, msg.sender_time) for msg in published],
                                     [(2, -1, 101.0)])
                finally:
                    stopped.set()
                    thread.join(2)
            self.assertEqual(failures, [])

    def test_ready_follows_waiting_and_completed_follows_result_and_delay(self):
        with tempfile.TemporaryDirectory() as directory:
            params = {'~joint_pose': {'joint': 0}, '~participant': 'P01', '~session': '0',
                      '~results_file': str(Path(directory) / 'results.csv')}
            callback = []
            observations = []
            ticks = []
            def sample(state):
                return types.SimpleNamespace(state=state, angle=0, elapsed=3.5)
            def sleep_tick():
                ticks.append(None)
                self.assertLess(len(ticks), 5, 'Controller never announces WAITING readiness')
                if not (Path(directory) / 'controller_ready').exists():
                    callback[0](sample(0))
                else:
                    callback[0](sample(2))
            def delay(seconds):
                observations.append('delay')
                self.assertTrue((Path(directory) / 'controller_ready').exists())
                self.assertFalse((Path(directory) / 'completed').exists())
                self.assertTrue((Path(directory) / 'results.csv').exists())
            rospy = types.SimpleNamespace(init_node=lambda *a: None,
                get_param=lambda name, default=None: params.get(name, default),
                Subscriber=lambda topic, kind, cb, **k: callback.append(cb),
                is_shutdown=lambda: False, Rate=lambda hz: types.SimpleNamespace(sleep=sleep_tick),
                sleep=delay, loginfo=lambda *a: None, logwarn=lambda *a: None,
                logerr=lambda *a: None)
            module = load_node('session_manager', rospy)
            def switch():
                self.assertFalse((Path(directory) / 'controller_ready').exists())
                self.assertIn('move', observations)
                return True
            def move(pose, max_dq):
                observations.append('move')
                return True
            with patch.dict(os.environ, {'FMP_STATUS_DIR': directory}), \
                    patch.object(module, 'wait_for_controllers', return_value=True), \
                    patch.object(module, 'move_to_start', side_effect=move), \
                    patch.object(module, 'switch_to_pour_controller', side_effect=switch):
                self.assertEqual(module.main(), 0)
            self.assertEqual(observations, ['move', 'delay'])
            self.assertTrue((Path(directory) / 'completed').exists(),
                            'Completed trial is not announced to supervisor')

    def test_failed_controller_switch_never_announces_ready_or_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            params = {'~joint_pose': {}, '~participant': 'P01', '~session': '0',
                      '~results_file': str(Path(directory) / 'results.csv')}
            rospy = types.SimpleNamespace(init_node=lambda *a: None,
                get_param=lambda name, default=None: params.get(name, default),
                Subscriber=lambda *a, **k: None, loginfo=lambda *a: None,
                logerr=lambda *a: None)
            module = load_node('session_manager', rospy)
            with patch.dict(os.environ, {'FMP_STATUS_DIR': directory}), \
                    patch.object(module, 'wait_for_controllers', return_value=True), \
                    patch.object(module, 'move_to_start', return_value=True), \
                    patch.object(module, 'switch_to_pour_controller', return_value=False):
                self.assertEqual(module.main(), 1)
            self.assertEqual(list(Path(directory).iterdir()), [])
