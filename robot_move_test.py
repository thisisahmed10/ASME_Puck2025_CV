"""
robot_move_test.py — standalone gpiozero test harness

This file runs a simple forward/backward motion test using gpiozero's
PWMOutputDevice on four PWM pins (BCM numbering). It intentionally does
not import or depend on the project's `motors` module so it can run
independently.

Usage example:
  python robot_move_test.py --direction forward --duration 1.5 --speed 60

Be careful: run only with hardware connected and supervision.
"""

import argparse
import sys
import time

try:
    from gpiozero import PWMOutputDevice
except Exception as e:
    PWMOutputDevice = None

# Default BCM pins (match your wiring)
LEFT_RPWM = 12
LEFT_LPWM = 13
RIGHT_RPWM = 18
RIGHT_LPWM = 19


class SimpleDrive:
    """Minimal differential-drive interface using gpiozero PWMOutputDevice.

    Each motor is controlled by two PWM pins (forward/reverse). Duty is
    given as 0..1.
    """

    def __init__(self, lr, ll, rr, rl):
        if PWMOutputDevice is None:
            raise ImportError("gpiozero.PWMOutputDevice is not available")
        self.lr = PWMOutputDevice(lr)
        self.ll = PWMOutputDevice(ll)
        self.rr = PWMOutputDevice(rr)
        self.rl = PWMOutputDevice(rl)

    def forward(self, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        self.lr.value = duty
        self.ll.value = 0
        self.rr.value = duty
        self.rl.value = 0

    def backward(self, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        self.lr.value = 0
        self.ll.value = duty
        self.rr.value = 0
        self.rl.value = duty

    def stop(self):
        for d in (self.lr, self.ll, self.rr, self.rl):
            try:
                d.value = 0
            except Exception:
                pass

    def close(self):
        for d in (self.lr, self.ll, self.rr, self.rl):
            try:
                d.close()
            except Exception:
                pass


def parse_args():
    p = argparse.ArgumentParser(description="Standalone gpiozero robot forward/backward test")
    p.add_argument("--direction", choices=("forward", "backward"), default="forward",
                   help="Direction to move")
    p.add_argument("--duration", type=float, default=1.0, help="Duration in seconds")
    p.add_argument("--speed", type=float, default=60.0, help="Speed 0..100 (percent)")
    p.add_argument("--pins", nargs=4, type=int, metavar=("LR","LL","RR","RL"),
                   help="Override BCM pins (left_r left_l right_r right_l)")
    return p.parse_args()


def main():
    args = parse_args()

    if PWMOutputDevice is None:
        print("ERROR: gpiozero is not installed or cannot be imported.\nInstall with: pip install gpiozero")
        sys.exit(2)

    pins = (LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM)
    if args.pins:
        if len(args.pins) != 4:
            print("--pins requires four integers: LR LL RR RL")
            sys.exit(2)
        pins = tuple(args.pins)

    drive = SimpleDrive(*pins)
    duty = max(0.0, min(1.0, args.speed / 100.0))

    print(f"Running: direction={args.direction} duration={args.duration}s speed={args.speed}% pins={pins}")

    try:
        if args.direction == "forward":
            drive.forward(duty)
        else:
            drive.backward(duty)
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("Interrupted — stopping")
    finally:
        drive.stop()
        drive.close()
        print("Done")


if __name__ == "__main__":
    main()
