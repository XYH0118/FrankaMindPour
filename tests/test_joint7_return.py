"""Compile and run the real C++ controller with fake ROS joints and commands."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Joint7ReturnTests(unittest.TestCase):
    def test_continuous_return_command_stops_at_start(self):
        compiler = os.environ.get('CXX') or shutil.which('c++') or shutil.which('cl')
        if not compiler:
            self.skipTest('A C++17 compiler (c++ or cl in a VS developer shell) is needed')

        with tempfile.TemporaryDirectory() as temporary:
            scratch = Path(temporary)
            stubs = ROOT / 'tests/fixtures/joint7_ros_stubs.h'
            (scratch / 'include').mkdir()
            shutil.copyfile(stubs, scratch / 'include/joint7_ros_stubs.h')
            for name in (
                'controller_interface/multi_interface_controller.h',
                'hardware_interface/joint_command_interface.h', 'hardware_interface/robot_hw.h',
                'realtime_tools/realtime_buffer.h', 'realtime_tools/realtime_publisher.h',
                'ros/node_handle.h', 'ros/time.h', 'ros/ros.h', 'pluginlib/class_list_macros.h',
                'franka_mind_pour/PourCommand.h', 'franka_mind_pour/PourState.h',
            ):
                header = scratch / 'include' / name
                header.parent.mkdir(parents=True, exist_ok=True)
                header.write_text('#include "joint7_ros_stubs.h"\n', encoding='ascii')

            includes = [scratch / 'include', ROOT / 'franka_mind_pour/include']
            sources = [ROOT / 'tests/fixtures/joint7_return.cpp',
                       ROOT / 'franka_mind_pour/src/mind_pour_controller.cpp']
            output = scratch / ('joint7_return.exe' if os.name == 'nt' else 'joint7_return')
            if Path(compiler).stem.lower() == 'cl':
                command = [compiler, '/nologo', '/std:c++17', '/EHsc', '/D_USE_MATH_DEFINES',
                           '/Fe:' + str(output)] + ['/I' + str(path) for path in includes]
            else:
                command = [compiler, '-std=c++17', '-o', str(output)] + [
                    '-I' + str(path) for path in includes]
            subprocess.run(command + [str(path) for path in sources], cwd=scratch, check=True)
            result = subprocess.run([str(output)], capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
