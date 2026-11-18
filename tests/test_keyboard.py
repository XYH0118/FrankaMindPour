"""Keyboard command mapping and managed experiment finish behavior."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import fmp_udp_keyboard as keyboard


class KeyboardTests(unittest.TestCase):
    def test_existing_focus_release_and_simultaneous_key_rules(self):
        class User32:
            window = 1
            keys = set()
            def GetForegroundWindow(self):
                return self.window
            def GetAsyncKeyState(self, key):
                return 0x8000 if key in self.keys else 0
        user32 = User32()
        read = keyboard.make_key_reader(user32)
        for keys, expected in [(set(), None), ({ord('1')}, '1'), ({ord('2')}, '2'),
                               ({ord('1'), ord('2')}, '0'), ({0x60, ord('1')}, '0')]:
            user32.keys = keys
            self.assertEqual(read(), expected)
        user32.window = 2
        user32.keys = {ord('1')}
        self.assertIsNone(read())

    def test_managed_key_reader_requests_finish_but_keeps_direction_control(self):
        self.assertTrue(hasattr(keyboard, 'managed_command_reader'),
                        'Keyboard has no TCP-managed control adapter')
        keys = iter(['q', None, '1', '2', '0', '\x1b'])
        read = keyboard.managed_command_reader(lambda: next(keys))
        self.assertEqual([read() for _ in range(6)], ['finish', 0, 1, -1, 0, 'finish'])


if __name__ == '__main__':
    unittest.main()
