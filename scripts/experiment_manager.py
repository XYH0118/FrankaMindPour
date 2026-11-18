#!/usr/bin/env python3
"""Persistent TCP trial supervisor; run on the Ubuntu robot host, outside ROS."""

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time


@dataclass(frozen=True)
class Request:
    request_id: str
    participant: str
    session: str


def trial_processes(directory, proc_root=Path('/proc')):
    """Find live Linux children by their exact inherited per-trial environment.

    ROS nodes can setsid() and outlive roslaunch. Process groups/PPIDs alone
    cannot identify those orphans. Empty environ files include exited zombies.
    """
    tag = os.fsencode('FMP_STATUS_DIR=' + str(directory))
    matches = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            environment = (entry / 'environ').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if tag in environment:
            matches.append(int(entry.name))
    return sorted(matches)


class SessionProcess:
    """Own a child process group, including roslaunch and its ROS nodes."""

    def __init__(self, command, directory, cwd=None):
        self.directory = str(directory)
        environment = os.environ.copy()
        environment['FMP_STATUS_DIR'] = str(directory)
        self.process = subprocess.Popen(command, cwd=cwd, env=environment,
                                        start_new_session=(os.name == 'posix'))

    def poll(self):
        return self.process.poll()

    def close(self, timeout):
        if sys.platform.startswith('linux'):
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                for pid in trial_processes(self.directory):
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    self.poll()
                    if not trial_processes(self.directory):
                        break
                    time.sleep(0.02)
                else:
                    continue
                break
            if trial_processes(self.directory):
                raise RuntimeError('Session cleanup failed: live trial processes remain')
        elif os.name == 'posix':
            # Also clean descendants if the wrapper has already exited.
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    self.process.poll()
                    try:
                        os.killpg(self.process.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.02)
                else:
                    continue
                break
        elif self.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=timeout)


class ExperimentManager:
    """Serialize trials and retain request outcomes across TCP reconnects."""

    def __init__(self, launch, startup_timeout=120.0, shutdown_timeout=5.0):
        self.launch = launch
        self.startup_timeout = startup_timeout
        self.shutdown_timeout = shutdown_timeout
        self.records = {}
        self.active = None
        self.peer = None
        self.finishing = False
        self.finished = False
        self.events = []

    def latest(self, request_id):
        return self.records[request_id]['status']

    def _status(self, record, state):
        request = record['request']
        record['status'] = '%s,%s,%s,%s' % (state, request.request_id,
                                          request.participant, request.session)
        return record['status']

    def handle(self, line, peer):
        parts = line.split(',')
        request_id = parts[1] if len(parts) > 1 and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', parts[1]) else '-'
        if self.peer is not None and peer != self.peer:
            return ['ERROR,%s,wrong_peer' % request_id]
        if parts == ['FINISH_EXPERIMENT']:
            self.finishing = True
            if self.active is None:
                self.finished = True
                return ['EXPERIMENT_FINISHED']
            return ['FINISHING']
        if len(parts) == 2 and parts[0] == 'STATUS' and request_id != '-':
            return [self.latest(request_id) if request_id in self.records
                    else 'ERROR,%s,unknown_request' % request_id]
        if (len(parts) != 4 or parts[0] != 'START' or request_id == '-'
                or not re.fullmatch(r'P[0-9]+', parts[2])
                or not re.fullmatch(r'0|[1-9][0-9]*', parts[3])):
            return ['ERROR,%s,invalid_request' % request_id]
        request = Request(*parts[1:])
        if request_id in self.records:
            previous = self.records[request_id]
            return [previous['status'] if previous['request'] == request
                    else 'ERROR,%s,request_conflict' % request_id]
        if self.finishing:
            return ['ERROR,%s,experiment_finishing' % request_id]
        if self.active is not None:
            return ['BUSY,%s' % request_id]
        self.peer = peer
        record = {'request': request}
        self.records[request_id] = record
        directory = tempfile.TemporaryDirectory(prefix='fmp_trial_')
        record['directory'] = directory
        try:
            record['process'] = self.launch(request, directory.name)
        except (OSError, ValueError) as error:
            directory.cleanup()
            print('Session launch failed: %s' % error, flush=True)
            record['status'] = 'ERROR,%s,launch_failed' % request_id
            return [record['status']]
        record['started'] = time.monotonic()
        record['ready'] = False
        self.active = record
        return [self._status(record, 'STARTING')]

    def tick(self):
        record = self.active
        if record is None:
            return
        directory = Path(record['directory'].name)
        exit_code = record['process'].poll()
        error = None
        if exit_code is None:
            if not record['ready']:
                if all((directory / name).exists() for name in ('controller_ready', 'bridge_ready')):
                    (directory / 'accept_commands').touch()
                    record['ready'] = True
                    self.events.append(self._status(record, 'READY'))
                elif time.monotonic() - record['started'] > self.startup_timeout:
                    error = 'startup_timeout'
            if error is None:
                return
        elif exit_code != 0:
            error = 'child_exit_%s' % exit_code
        elif not record['ready'] or not (directory / 'completed').exists():
            error = 'incomplete_trial'
        record['process'].close(self.shutdown_timeout)
        record['directory'].cleanup()
        if error:
            record['status'] = 'ERROR,%s,%s' % (record['request'].request_id, error)
        else:
            self._status(record, 'DONE')
        self.events.append(record['status'])
        self.active = None
        if self.finishing:
            self.finished = True
            self.events.append('EXPERIMENT_FINISHED')

    def drain_events(self):
        events, self.events = self.events, []
        return events

    def close(self):
        if self.active is not None:
            self.active['process'].close(self.shutdown_timeout)
            self.active['directory'].cleanup()
            self.active = None


class ExperimentServer:
    """Single-client newline-framed TCP transport with bounded input."""

    def __init__(self, manager, host='0.0.0.0', port=5006):
        self.manager = manager
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(4)
        self.address = self.listener.getsockname()
        self.client = None
        self.peer = None
        self.buffer = b''
        self.closed = False

    def _disconnect(self):
        if self.client is not None:
            self.client.close()
        self.client = None
        self.buffer = b''

    def _send(self, lines):
        if self.client is not None and lines:
            try:
                self.client.sendall(('\n'.join(lines) + '\n').encode('ascii'))
            except OSError:
                self._disconnect()

    def serve(self):
        print('Experiment TCP listening on %s:%s' % self.address, flush=True)
        try:
            while not self.closed and not self.manager.finished:
                self.manager.tick()
                self._send(self.manager.drain_events())
                if self.manager.finished:
                    break
                readers = [self.listener] + ([self.client] if self.client else [])
                try:
                    readable, _, _ = select.select(readers, [], [], 0.02)
                except (OSError, ValueError):
                    if self.closed:
                        break
                    raise
                if self.listener in readable:
                    connection, address = self.listener.accept()
                    connection.settimeout(1.0)
                    if self.client is not None or (self.manager.peer and address[0] != self.manager.peer):
                        try:
                            connection.sendall(b'ERROR,-,connection_busy\n')
                        except OSError:
                            pass
                        finally:
                            connection.close()
                    else:
                        self.client, self.peer = connection, address[0]
                if self.client is not None and self.client in readable:
                    try:
                        data = self.client.recv(4096)
                    except OSError:
                        data = b''
                    if not data:
                        self._disconnect()
                        continue
                    self.buffer += data
                    while b'\n' in self.buffer and self.client is not None:
                        line, self.buffer = self.buffer.split(b'\n', 1)
                        try:
                            if len(line) >= 1024:
                                raise ValueError('line too long')
                            decoded = line.decode('ascii').rstrip('\r')
                        except (UnicodeDecodeError, ValueError):
                            self._send(['ERROR,-,invalid_frame'])
                            self._disconnect()
                            break
                        self._send(self.manager.handle(decoded, self.peer))
                        if self.manager.finished:
                            break
                    if len(self.buffer) >= 1024:
                        self._send(['ERROR,-,invalid_frame'])
                        self._disconnect()
        finally:
            self.close()

    def close(self):
        self.closed = True
        self._disconnect()
        self.listener.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5006)
    parser.add_argument('--startup-timeout', type=float, default=120.0)
    parser.add_argument('--robot-ip', default='192.168.1.11')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('port must be between 1 and 65535')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('startup timeout must be finite and positive')
    root = Path(__file__).resolve().parents[1]

    def launch(request, directory):
        return SessionProcess(['bash', str(root / 'scripts/run_session.sh'),
                               'participant:=' + request.participant,
                               'session:=' + request.session, 'robot_ip:=' + args.robot_ip],
                              directory, cwd=str(root))

    manager = ExperimentManager(launch, startup_timeout=args.startup_timeout)
    server = ExperimentServer(manager, args.host, args.port)
    try:
        server.serve()
    except KeyboardInterrupt:
        print('Stopping experiment and child session...', flush=True)
    finally:
        server.close()
        manager.close()


if __name__ == '__main__':
    main()
