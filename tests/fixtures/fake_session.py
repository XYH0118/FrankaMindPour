"""Robot-free subprocess fixture for supervisor integration tests."""
import os
from pathlib import Path
import sys
import time
import subprocess

directory = Path(os.environ['FMP_STATUS_DIR'])
mode = sys.argv[1]
if mode == 'detached':
    descendant = subprocess.Popen([sys.executable, '-c',
        'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); '
        'signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)'], start_new_session=True)
    (directory / 'descendant').write_text(str(descendant.pid))
    # Give the detached child time to install signal handlers before exiting.
    time.sleep(0.1)
    sys.exit(0)
if mode == 'fail':
    sys.exit(7)
if mode == 'hang':
    time.sleep(60)
    sys.exit(1)
if mode in ('move', 'move_incomplete'):
    while not (directory / 'release').exists():
        time.sleep(0.01)
    if mode == 'move':
        (directory / 'completed').touch()
    sys.exit(0)
(directory / 'controller_ready').touch()
time.sleep(0.08)
(directory / 'bridge_ready').touch()
while not (directory / 'accept_commands').exists():
    time.sleep(0.01)
while not (directory / 'release').exists():
    time.sleep(0.01)
if mode != 'incomplete':
    (directory / 'completed').touch()
sys.exit(5 if mode == 'bad_exit' else 0)
