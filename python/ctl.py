#!/usr/bin/env python3

# User control of car: arrows = speed/turn, Ctrl+C (or q) to stop

import select, socket, sys, termios, tty

import constants

STATUS_POLL_S = 0.2  # how often status() is re-checked while no key is pressed


def turn_label(t):
    if t == 0:
        return "center"
    return f"{abs(t)}° {'left' if t < 0 else 'right'}"


def run(status=None):
    # status: optional callable returning extra text for the drive line (e.g. "AVOID" / "LIDAR STOP")
    drive = socket.create_connection(("127.0.0.1", constants.ESC_PORT))  # motor.py
    steer = socket.create_connection(("127.0.0.1", constants.STEERING_SERVO_PORT))  # steering.py

    speed = 0
    turn = 0  # Value between [-180, 180], for full left and full right
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    last_status = None

    def show():
        extra = status() if status else ""
        print(f"Speed {speed}% | Turning {turn_label(turn)}" + (f" | {extra}" if extra else ""))
        return extra

    try:
        tty.setcbreak(fd)
        print("Arrows: up/down speed, left/right turn. Ctrl+C to stop.")
        while True:
            if status and not select.select([sys.stdin], [], [], STATUS_POLL_S)[0]:
                extra = status()
                if extra != last_status:
                    last_status = show()
                continue
            ch = sys.stdin.read(1)
            if ch == "q":
                break
            if ch == "\x1b" and sys.stdin.read(1) == "[":
                k = sys.stdin.read(1)
                if k == "A": speed = min(constants.CONTROL_SPEED_MAX, speed + 1)
                elif k == "B": speed = max(constants.CONTROL_SPEED_MIN, speed - 1)
                elif k == "C": turn = min(constants.CONTROL_TURN_MAX, turn + 5)
                elif k == "D": turn = max(constants.CONTROL_TURN_MIN, turn - 5)
                else: continue
                if k in "AB":
                    drive.sendall(f"{speed}\n".encode())
                else:
                    steer.sendall(f"{turn}\n".encode())
                last_status = show()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        drive.sendall(b"0\n")
        steer.sendall(b"0\n")
        drive.close()
        steer.close()


if __name__ == "__main__":
    run()
