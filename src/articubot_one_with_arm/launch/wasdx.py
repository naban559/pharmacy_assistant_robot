#!/usr/bin/env python3
"""
Simple w/a/s/d/x keyboard teleop for the ROSArduinoBridge firmware.

This talks directly over USB serial using the sketch's existing command
protocol - it doesn't care whether the L298 or BTS7960 driver is selected
on the Arduino side, and it doesn't need ROS at all.

Controls:
  w = forward
  s = backward
  a = turn left in place
  d = turn right in place
  x = stop
  + / - = increase / decrease speed
  q = quit

Sends the MOTOR_RAW_PWM ('o') command, e.g. "o 120 120\r", which bypasses
the PID loop and drives the wheels at a fixed PWM value (-255..255).
"""

import sys
import time
import argparse
import serial

IS_WINDOWS = sys.platform.startswith('win')
if IS_WINDOWS:
    import msvcrt
else:
    import tty
    import termios
    import select

HELP = """
Controls:
w/s : forward / backward
a/d : turn left / turn right
x   : stop
+/- : speed up / speed down
q   : quit

Single Motor Test Keys:
1 : Left motor ONLY Forward  (o 120 0)
2 : Left motor ONLY Reverse  (o -120 0)
3 : Right motor ONLY Forward (o 0 120)
4 : Right motor ONLY Reverse (o 0 -120)
"""


def get_key(timeout, old_settings=None):
    """Return a single keypress, or '' if nothing was pressed within timeout."""
    if IS_WINDOWS:
        start = time.time()
        while (time.time() - start) < timeout:
            if msvcrt.kbhit():
                return msvcrt.getwch()
            time.sleep(0.01)
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key


def send_pwm(ser, left, right):
    left = max(-255, min(255, int(left)))
    right = max(-255, min(255, int(right)))
    ser.write(f"o {left} {right}\r".encode())


def main():
    parser = argparse.ArgumentParser(description="WASDX keyboard teleop for ROSArduinoBridge")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. COM5 or /dev/ttyUSB1 (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=57600, help="Must match BAUDRATE in the sketch")
    parser.add_argument("--speed", type=int, default=120, help="Starting PWM magnitude (0-255)")
    parser.add_argument("--step", type=int, default=20, help="Speed change per +/- keypress")
    parser.add_argument("--release-timeout", type=float, default=0.15,
                         help="Seconds with no repeated key before auto-stopping")
    args = parser.parse_args()

    speed = args.speed
    old_settings = None if IS_WINDOWS else termios.tcgetattr(sys.stdin)

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2)  # let the Arduino finish its auto-reset after the port opens
    print(HELP)

    moving = False
    try:
        while True:
            key = get_key(args.release_timeout, old_settings)

            if key == 'w':
                send_pwm(ser, speed, speed)
                moving = True
            elif key == 's':
                send_pwm(ser, -speed, -speed)
                moving = True
            elif key == 'a':
                send_pwm(ser, -speed, speed)
                moving = True
            elif key == 'd':
                send_pwm(ser, speed, -speed)
                moving = True
            elif key == '1':
                send_pwm(ser, speed, 0)
                moving = True
            elif key == '2':
                send_pwm(ser, -speed, 0)
                moving = True
            elif key == '3':
                send_pwm(ser, 0, speed)
                moving = True
            elif key == '4':
                send_pwm(ser, 0, -speed)
                moving = True
            elif key == 'x':
                send_pwm(ser, 0, 0)
                moving = False
            elif key in ('+', '='):
                speed = min(255, speed + args.step)
                print(f"speed = {speed}")
            elif key in ('-', '_'):
                speed = max(0, speed - args.step)
                print(f"speed = {speed}")
            elif key == 'q':
                break
            elif key == '\x03':  # Ctrl-C
                break
            elif key == '':
                # no key repeated within release_timeout -> treat as released
                if moving:
                    send_pwm(ser, 0, 0)
                    moving = False
    finally:
        send_pwm(ser, 0, 0)
        ser.close()
        if not IS_WINDOWS:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nStopped, serial closed.")


if __name__ == "__main__":
    main()