#!/usr/bin/env python3
"""Windows multi-trial TCP/UDP keyboard test: 0=NONE, 1=clockwise, 2=anti-clockwise.

Hold 1 or 2 to send motion; releasing the key sends NONE continuously.
Uses the original fmp_udp_sender ASCII protocol and standard library.
Packets are "sequence,command,sender_time" followed by a newline; command is 0, 1 or -1.
Keyboard 2 maps to command -1. Default destination: 192.168.50.2:5005 at 50 Hz.
"""

import math
import sys
import time

from fmp_udp_sender import (ANTI_CLOCKWISE, CLOCKWISE, NONE, CommandSender,
                            argument_parser, managed_experiment, validate_arguments)


KEY_COMMANDS = {'0': NONE, '1': CLOCKWISE, '2': ANTI_CLOCKWISE}
COMMAND_NAMES = {NONE: 'NONE / 无方向信号', CLOCKWISE: '顺时针', ANTI_CLOCKWISE: '逆时针'}


def make_key_reader(user32=None):
    """Poll physical key state, accepting input only in the starting window."""
    if user32 is None:
        import ctypes
        from ctypes import wintypes

        user32 = ctypes.WinDLL('user32', use_last_error=True)
        user32.GetForegroundWindow.argtypes = []
        user32.GetForegroundWindow.restype = wintypes.HWND
        user32.GetAsyncKeyState.argtypes = [ctypes.c_int]
        user32.GetAsyncKeyState.restype = ctypes.c_short
    control_window = user32.GetForegroundWindow()

    def read_key():
        if not control_window or user32.GetForegroundWindow() != control_window:
            return None

        def held(key):
            return bool(user32.GetAsyncKeyState(key) & 0x8000)

        if held(ord('Q')):
            return 'q'
        if held(0x1b):
            return '\x1b'
        zero = held(ord('0')) or held(0x60)
        clockwise = held(ord('1')) or held(0x61)
        anti_clockwise = held(ord('2')) or held(0x62)
        if zero or (clockwise and anti_clockwise):
            return '0'
        if clockwise:
            return '1'
        if anti_clockwise:
            return '2'
        return None

    return read_key


def run_keyboard(sender, read_key, rate=50.0):
    """Send the held command, or NONE when read_key reports no held digit."""
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('rate must be a finite positive number')
    period = 1.0 / rate
    command = NONE
    next_send = time.perf_counter()
    try:
        while True:
            key = read_key()
            if key in ('q', 'Q', '\x1b', '\x03'):
                break
            selected = KEY_COMMANDS.get(key, NONE)
            if selected != command:
                print(f'当前 UDP 指令值: {selected} ({COMMAND_NAMES[selected]})', flush=True)
            command = selected
            sender.send(command)
            next_send += period
            now = time.perf_counter()
            if next_send <= now:
                # Resume at the current time after a stall; never replay a backlog.
                next_send = now + period
            time.sleep(next_send - now)
    finally:
        # Repeat NONE on orderly exits and exceptions; UDP has no delivery ACK.
        for _ in range(3):
            try:
                sender.send(NONE)
            except OSError as exc:
                print(f'发送停止指令失败: {exc}', file=sys.stderr, flush=True)
            time.sleep(period)


def managed_command_reader(read_key):
    """Keep legacy digit mapping; Q/Esc finish after the current trial."""
    def read_command():
        key = read_key()
        if key in ('q', 'Q', '\x1b'):
            return 'finish'
        return KEY_COMMANDS.get(key, NONE)
    return read_command


def main():
    parser = argument_parser(__doc__)
    args = parser.parse_args()
    validate_arguments(parser, args)
    if sys.platform != 'win32':
        parser.error('请在 Windows 终端中运行此键盘测试脚本')
    if not sys.stdin.isatty():
        parser.error('需要交互式终端；请双击 fmp_udp_keyboard.bat 或在 PowerShell 中运行')

    print(f'UDP 目标: {args.host}:{args.port} | {args.rate:g} 包/秒', flush=True)
    print('按键 0 -> 发送 0 (NONE)    按键 1 -> 发送 1 (顺时针)    按键 2 -> 发送 -1 (逆时针)',
          flush=True)
    print(r'UDP 数据包: 序号,指令值,发送时间戳\n（ASCII；序号和时间戳自动生成）', flush=True)
    print('按住 1 或 2 才发送方向信号；松开立即恢复连续发送 0，无需回车。', flush=True)
    print('同时按 1 和 2、按住 0 或切走窗口时，均发送 0。', flush=True)
    if args.udp_only:
        print('UDP 调试模式：Q / Esc / Ctrl+C 退出。', flush=True)
    else:
        print(f'TCP 管理端: {args.host}:{args.tcp_port}；输入编号后等待 READY，再开放按键控制。', flush=True)
        print('Q / Esc 请求本轮结束后退出全部实验；之后仍可按 1/2 完成回位。', flush=True)
        print('Ctrl+C 退出本地客户端并尝试发送停止指令；不会自动终止 Ubuntu 管理程序。', flush=True)
    sender = None
    exit_code = 0
    try:
        if not args.udp_only:
            def command_factory():
                return managed_command_reader(make_key_reader())
            return managed_experiment(args, command_factory)
        sender = CommandSender(args.host, args.port)
        run_keyboard(sender, make_key_reader(), args.rate)
    except (KeyboardInterrupt, EOFError):
        print('本地客户端已退出；Ubuntu 上未完成的任务不会自动报告成功。', flush=True)
        exit_code = 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'通信失败: {exc}', file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        if sender is not None:
            sender._socket.close()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
