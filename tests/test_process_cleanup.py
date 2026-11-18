"""Linux process-tag selection tested without touching real processes."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import experiment_manager


class CleanupTests(unittest.TestCase):
    def test_only_exact_trial_environment_is_selected_including_detached_children(self):
        self.assertTrue(hasattr(experiment_manager, 'trial_processes'),
                        'Detached ROS children cannot be found for cleanup')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for pid, environment in [('101', b'FMP_STATUS_DIR=/trial/a\0OTHER=x\0'),
                                     ('102', b'FMP_STATUS_DIR=/trial/ab\0'),
                                     ('103', b'OTHER=FMP_STATUS_DIR=/trial/a\0'),
                                     ('104', b'FMP_STATUS_DIR=/trial/a\0')]:
                directory = root / pid
                directory.mkdir()
                (directory / 'environ').write_bytes(environment)
            (root / '104' / 'environ').unlink()
            self.assertEqual(experiment_manager.trial_processes('/trial/a', root), [101])

    def test_cleanup_failure_prevents_a_new_trial(self):
        self.assertTrue(hasattr(experiment_manager, 'trial_processes'))
        # A surviving ROS descendant must raise, never silently permit DONE.
        process = object.__new__(experiment_manager.SessionProcess)
        process.directory = '/trial/a'
        class Wrapper:
            pid = 123
            def poll(self):
                return 0
        process.process = Wrapper()
        with patch.object(experiment_manager.sys, 'platform', 'linux'), \
                patch.object(experiment_manager, 'trial_processes', return_value=[456]), \
                patch.object(experiment_manager.signal, 'SIGKILL', 9, create=True), \
                patch.object(experiment_manager.os, 'kill'):
            with self.assertRaisesRegex(RuntimeError, 'cleanup'):
                process.close(0.01)
