#!/usr/bin/env python3
"""TCP experiment client and compatible UDP sender; standard library only.

Default CLI: move to the start pose, prompt for participant/session, wait for
READY, play a pattern, wait for DONE, then prompt for the next trial. Enter Q
at an ID prompt to finish.
Use --udp-only for the original UDP-only debug behavior. CommandSender's API
and ASCII "<seq>,<command>,<sender_time>\\n" datagrams remain unchanged.
"""

import argparse
import math
import re
import select
import socket
import sys
import time
import uuid

CLOCKWISE = 1
NONE = 0
ANTI_CLOCKWISE = -1


class CommandSender:
    """Sends one command per call. Not thread-safe; use one per thread."""

    def __init__(self, host, port=5005):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0

    def send(self, command):
        if command not in (ANTI_CLOCKWISE, NONE, CLOCKWISE):
            raise ValueError('command must be -1, 0 or 1, got %r' % command)
        self._seq += 1
        packet = '%d,%d,%.6f\n' % (self._seq, command, time.time())
        self._socket.sendto(packet.encode('ascii'), self._address)


class ExperimentClient:
    """One TCP management connection; call poll regularly while sending UDP."""

    def __init__(self, host, port=5006, timeout=5.0):
        self._socket = socket.create_connection((host, port), timeout=timeout)
        self._buffer = b''
        self.finish_requested = False
        self.experiment_finished = False

    def _send(self, line):
        self._socket.sendall((line + '\n').encode('ascii'))

    def move_to_start(self, timeout):
        request_id = uuid.uuid4().hex
        print('已连接 Ubuntu，正在请求机械臂移动到起始姿态。', flush=True)
        self._send('MOVE_TO_START,' + request_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self.poll(max(0, deadline - time.monotonic())):
                fields = line.split(',', 2)
                if len(fields) < 2 or fields[1] not in (request_id, '-'):
                    continue
                if fields[0] == 'AT_START':
                    print('机械臂已到达起始姿态。', flush=True)
                    return
                if fields[0] == 'BUSY':
                    raise RuntimeError('Ubuntu 上已有动作进行中。请等待结束或恢复原任务。')
                if fields[0] == 'ERROR':
                    if fields[1:] == [request_id, 'invalid_request']:
                        raise RuntimeError('Ubuntu 管理程序需要更新，才能执行连接后回位。')
                    raise RuntimeError('起始姿态移动失败 (%s)' % line)
        raise TimeoutError('等待机械臂到达起始姿态超时 (%s)' % request_id)

    def start(self, participant, session, request_id=None):
        session = str(session)
        if not re.fullmatch(r'P[0-9]+', participant):
            raise ValueError('受试者编号应为 P 加数字，例如 P01')
        if not re.fullmatch(r'0|[1-9][0-9]*', session):
            raise ValueError('session 应为非负整数，例如 0、1、2')
        request_id = request_id or uuid.uuid4().hex
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', request_id):
            raise ValueError('request_id 应为 1–64 个字母、数字、下划线或连字符')
        self._send('START,%s,%s,%s' % (request_id, participant, session))
        return request_id

    def status(self, request_id):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', request_id):
            raise ValueError('Invalid request ID')
        self._send('STATUS,' + request_id)

    def finish(self):
        if not self.finish_requested:
            self._send('FINISH_EXPERIMENT')
            self.finish_requested = True

    def poll(self, timeout=0.0):
        readable, _, _ = select.select([self._socket], [], [], timeout)
        if not readable:
            return []
        data = self._socket.recv(4096)
        if not data:
            raise ConnectionError('TCP 连接已断开；请勿用新的 request_id 重复启动尚未确认结束的任务')
        self._buffer += data
        lines = []
        while b'\n' in self._buffer:
            line, self._buffer = self._buffer.split(b'\n', 1)
            if len(line) >= 1024:
                raise ValueError('管理端返回的消息过长')
            decoded = line.decode('ascii').rstrip('\r')
            lines.append(decoded)
            if decoded == 'EXPERIMENT_FINISHED':
                self.experiment_finished = True
        if len(self._buffer) >= 1024:
            raise ValueError('管理端返回的消息过长')
        return lines

    def close(self):
        self._socket.close()


def run_managed_trial(client, sender, request_id, read_command, rate=50.0,
                      on_status=print, startup_timeout=150.0, trial_timeout=None):
    """Wait for READY, then send commands until DONE/ERROR, never before READY.

    read_command returns -1/0/1, or 'finish' to request graceful experiment
    finish while retaining control until this trial is complete.
    """
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('rate must be finite and positive')
    ready = False
    start = time.monotonic()
    ready_time = None
    next_send = start
    period = 1.0 / rate
    try:
        while True:
            wait = min(period, max(0, next_send - time.monotonic())) if ready else 0.05
            for line in client.poll(wait):
                on_status(line)
                fields = line.split(',')
                if fields[0] == 'ERROR' and len(fields) > 1 and fields[1] == '-':
                    return 'ERROR'
                if len(fields) < 2 or fields[1] != request_id:
                    continue
                if fields[0] == 'READY' and not ready:
                    ready = True
                    ready_time = next_send = time.monotonic()
                elif fields[0] in ('DONE', 'ERROR', 'BUSY'):
                    return fields[0]
            now = time.monotonic()
            if not ready:
                if now - start > startup_timeout:
                    raise TimeoutError('等待 READY 超时；请查询原 request_id，勿重复启动')
                continue
            if trial_timeout is not None and now - ready_time > trial_timeout:
                raise TimeoutError('等待本轮结束超时，已停止发送方向指令')
            if now >= next_send:
                command = read_command()
                if command == 'finish':
                    if not client.finish_requested:
                        client.finish()
                        on_status('已请求结束全部实验；本轮仍可用方向键完成回位。')
                    command = NONE
                sender.send(command)
                # Skip missed deadlines instead of bursting stale commands.
                next_send = max(next_send + period, time.monotonic())
    finally:
        if ready:
            for _ in range(3):
                try:
                    sender.send(NONE)
                except OSError:
                    break
                time.sleep(period)


def parse_pattern(text):
    """'1:2,0:1' -> [(1, 2.0), (0, 1.0)]: command, then how long to send it [s]."""
    steps = []
    for item in text.split(','):
        command, duration = item.split(':')
        command, duration = int(command), float(duration)
        if command not in (ANTI_CLOCKWISE, NONE, CLOCKWISE):
            raise ValueError('pattern command must be -1, 0 or 1')
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('pattern duration must be finite and positive')
        steps.append((command, duration))
    return steps


def pattern_reader(steps):
    started = None
    def read_command():
        nonlocal started
        if started is None:
            started = time.monotonic()
        elapsed = time.monotonic() - started
        for command, duration in steps:
            if elapsed < duration:
                return command
            elapsed -= duration
        return NONE
    return read_command


def argument_parser(description=__doc__):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--host', default='192.168.50.2', help='Ubuntu PC address')
    parser.add_argument('--port', type=int, default=5005, help='UDP command port')
    parser.add_argument('--tcp-port', type=int, default=5006)
    parser.add_argument('--rate', type=float, default=50.0, help='packets per second')
    parser.add_argument('--participant', help='e.g. P01; otherwise prompt')
    parser.add_argument('--session', help='e.g. 0; otherwise prompt')
    parser.add_argument('--request-id', help='reuse the original ID to recover a disconnected trial')
    parser.add_argument('--startup-timeout', type=float, default=150.0)
    parser.add_argument('--trial-timeout', type=float, help='optional seconds after READY')
    parser.add_argument('--once', action='store_true', help='exit after one trial; leave manager listening')
    parser.add_argument('--udp-only', action='store_true', help='legacy UDP-only debug mode')
    parser.add_argument('--finish', action='store_true', help='request experiment finish without starting a trial')
    parser.add_argument('--status', metavar='REQUEST_ID', help='query a request without sending UDP')
    return parser


def validate_arguments(parser, args):
    if not 1 <= args.port <= 65535 or not 1 <= args.tcp_port <= 65535:
        parser.error('ports must be between 1 and 65535')
    for name in ('rate', 'startup_timeout', 'trial_timeout'):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error('%s must be finite and positive' % name)
    if args.udp_only and (args.finish or args.status or args.request_id):
        parser.error('--udp-only cannot be combined with management options')


def wait_experiment_finished(client):
    successful = True
    while not client.experiment_finished:
        for line in client.poll(1.0):
            print(line, flush=True)
            if line.startswith('ERROR,'):
                if line.startswith('ERROR,-,'):
                    raise RuntimeError(line)
                successful = False
    return successful


def managed_experiment(args, command_factory):
    """Shared multi-trial prompt loop for pattern and keyboard clients."""
    client = ExperimentClient(args.host, args.tcp_port)
    sender = CommandSender(args.host, args.port)
    try:
        if args.status:
            client.status(args.status)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                for line in client.poll(max(0, deadline - time.monotonic())):
                    print(line, flush=True)
                    fields = line.split(',')
                    if len(fields) > 1 and fields[1] in (args.status, '-'):
                        return 1 if fields[0] == 'ERROR' else 0
            raise TimeoutError('未收到完整的状态查询回复')
        if args.finish:
            client.finish()
            return 0 if wait_experiment_finished(client) else 1
        participant, session, request_id = args.participant, args.session, args.request_id
        if request_id is None:
            client.move_to_start(args.startup_timeout)
        while True:
            participant = participant or input('受试者编号（如 P01；Q 结束全部实验）: ').strip()
            if participant.lower() in ('q', 'quit', 'finish'):
                client.finish()
                return 0 if wait_experiment_finished(client) else 1
            session = session if session is not None else input('session 编号（0、1、2…；Q 结束）: ').strip()
            if session.lower() in ('q', 'quit', 'finish'):
                client.finish()
                return 0 if wait_experiment_finished(client) else 1
            try:
                request_id = client.start(participant, session, request_id)
            except ValueError as error:
                print(error, flush=True)
                participant = session = request_id = None
                continue
            print('请求 %s: %s / session %s；正在等待机械臂准备完成…' %
                  (request_id, participant, session), flush=True)
            result = run_managed_trial(client, sender, request_id, command_factory(),
                                       rate=args.rate, startup_timeout=args.startup_timeout,
                                       trial_timeout=args.trial_timeout,
                                       on_status=lambda line: print(line, flush=True))
            if client.finish_requested:
                wait_experiment_finished(client)
                return 0 if result == 'DONE' else 1
            if result == 'BUSY':
                print('管理端仍有任务运行；请查询原请求，不会自动排队或启动。', flush=True)
                return 1
            if args.once:
                return 0 if result == 'DONE' else 1
            if sys.platform == 'win32':
                import msvcrt
                while msvcrt.kbhit():
                    msvcrt.getwch()
            participant = session = request_id = None
    finally:
        sender._socket.close()
        client.close()


def main():
    parser = argument_parser()
    parser.add_argument('--pattern', default='0:1,1:2,0:1,-1:2,0:2',
                        help='comma-separated command:seconds steps; the default turns '
                             'clockwise for 2 s, then back for 2 s, which ends a session')
    args = parser.parse_args()
    validate_arguments(parser, args)
    try:
        steps = parse_pattern(args.pattern)
    except ValueError as error:
        parser.error(str(error))
    try:
        if not args.udp_only:
            return managed_experiment(args, lambda: pattern_reader(steps))
        sender = CommandSender(args.host, args.port)
        try:
            read_command = pattern_reader(steps)
            end = time.monotonic() + sum(duration for _, duration in steps)
            while time.monotonic() < end:
                sender.send(read_command())
                time.sleep(1.0 / args.rate)
        finally:
            for _ in range(3):
                sender.send(NONE)
                time.sleep(1.0 / args.rate)
            sender._socket.close()
    except (OSError, ValueError, RuntimeError) as error:
        print('通信失败: %s' % error, file=sys.stderr, flush=True)
        return 1
    except (KeyboardInterrupt, EOFError):
        print('\n本地客户端已退出；这不会自动宣告 Ubuntu 上的实验成功或结束。', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
