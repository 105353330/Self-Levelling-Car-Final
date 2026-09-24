# Entry point, two roles:
#   service - App Lab starts this with no terminal: runs every Bridge module's begin()/update() via App.run
#   menu    - `docker exec -it rc-car-main-1 python3 /app/python/main.py` (has a terminal): text menu that
#             drives the service over localhost sockets. Ctrl+C in any option returns to the menu.
# Force a role with --service / --menu.

import importlib
import math
import re
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import constants

PYTHON_DIR = Path(__file__).resolve().parent
APP_DIR = PYTHON_DIR.parent
CONSTANTS_H = APP_DIR / "sketch" / "constants.h"
AVOIDANCE_LOG = PYTHON_DIR / "avoidance.log"
CAMERA_NOT_FOUND_EXIT = 3  # same as objectAvoidance.CAMERA_NOT_FOUND_EXIT (not imported: it pulls in torch)
YOLO_MISSING_EXIT = 4      # same as objectAvoidance.YOLO_MISSING_EXIT

LEVEL_PARAM_NAMES = [
    "LEVEL_ROLL_ZERO_DEG", "LEVEL_PITCH_ZERO_DEG",
    "LEVEL_INV_ROLL_0", "LEVEL_INV_ROLL_1", "LEVEL_INV_ROLL_2",
    "LEVEL_INV_PITCH_0", "LEVEL_INV_PITCH_1", "LEVEL_INV_PITCH_2",
]  # same order as control.py / PlatformLevel::setParam

DATA_INTERVAL_S = 0.2
LEVEL_PRINT_INTERVAL_S = 0.5
CAL_POLL_S = 0.5
CAL_START_TIMEOUT_S = 5.0


# ---------------------------------------------------------------- service role

def service():
    from arduino.app_utils import App

    import avoidanceOverride
    import control
    import lidarOverride
    import motor
    import pwmCalibration
    import steering

    avoidanceOverride.begin()
    motor.begin()
    steering.begin()
    pwmCalibration.begin()
    lidarOverride.begin()
    control.begin()

    period = 1 / constants.SERVICE_LOOP_HZ
    next_tick = time.monotonic()

    def loop():
        nonlocal next_tick
        control.update()
        motor.update()
        steering.update()
        pwmCalibration.update()
        lidarOverride.update()
        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()  # fell behind: don't try to catch up in a burst

    App.run(user_loop=loop)


# ---------------------------------------------------------------- menu role

class Link:
    # Line request/reply connection to control.py
    def __init__(self):
        self._sock = socket.create_connection(("127.0.0.1", constants.CONTROL_PORT))
        self._file = self._sock.makefile()

    def ask(self, line):
        self._sock.sendall(f"{line}\n".encode())
        try:
            return self._file.readline().strip()
        except KeyboardInterrupt:
            self._file.readline()  # drain this request's reply so the next ask() stays in step
            raise

    def floats(self, line):
        # Reply as floats, or None if the MCU hasn't produced data yet
        try:
            return [float(v) for v in self.ask(line).split()]
        except ValueError:
            return None

    def mode(self, mode):
        self.ask(f"MODE {mode}")

    def close(self):
        self._sock.close()

    def diagnose(self):
        # Why is there no accelerometer data? Compares the MCU loop counter across two reads.
        first = self.ask("STATUS").split()
        time.sleep(0.3)
        second = self.ask("STATUS").split()
        if len(first) != 2 or len(second) != 2 or "None" in first + second:
            return "no reply from MCU (sketch not uploaded / Bridge down?)"
        if first[0] == second[0]:
            return "MCU loop is stuck (a sensor read is blocking loop(), check the lidar on SDA/SCL)"
        if second[1] != "1":
            return "accelerometer not responding on Qwiic (Wire1, address 0x1C), check wiring"
        return "accelerometer configured, waiting for first reading"


def roll_pitch(gx, gy, gz):
    # Same maths as platformLevel.cpp: tilt from gravity minus the level reference
    roll = math.degrees(math.atan2(gx, gz)) - constants.LEVEL_ROLL_ZERO_DEG
    pitch = math.degrees(math.atan2(gy, gz)) - constants.LEVEL_PITCH_ZERO_DEG
    return roll, pitch


# ---- 1. Driving

def _start_avoidance():
    try:
        log = open(AVOIDANCE_LOG, "w")
        proc = subprocess.Popen(
            [sys.executable, str(PYTHON_DIR / "objectAvoidance.py")],
            cwd=APP_DIR, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,  # our Ctrl+C must not reach it; stopped explicitly below
        )
        log.close()
        return proc
    except OSError as e:
        print(f"Camera avoidance not started: {e}")
        return None


def _stop_avoidance(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)  # lets objectAvoidance.py's finally release the camera and clear the override
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def drive(link):
    import ctl

    link.mode(constants.MODE_DRIVE)
    print(f"Driving (platform levelling + lidar stop on). Camera avoidance log: {AVOIDANCE_LOG}")
    avoid = _start_avoidance()

    def status():
        parts = []
        code = None if avoid is None else avoid.poll()
        if avoid is None:
            parts.append("camera avoidance OFF")
        elif code == CAMERA_NOT_FOUND_EXIT:
            parts.append("camera not found, avoidance OFF")
        elif code == YOLO_MISSING_EXIT:
            parts.append("YOLO not installed, avoidance OFF")
        elif code is not None:
            parts.append(f"camera avoidance stopped, see {AVOIDANCE_LOG.name}")
        elif link.ask("AVOID") == "1":
            parts.append("AVOID")
        lidar = link.ask("LIDAR").split()
        if len(lidar) == 3 and lidar[1] == "1":
            parts.append("LIDAR STOP")
        return " | ".join(parts)

    try:
        ctl.run(status=status)
    finally:
        _stop_avoidance(avoid)


# ---- 2. PWM calibration

def pwm_calibration(link):
    import pwmCtl

    link.mode(constants.MODE_PWM_CAL)
    print("PWM calibration: motor, steering and platform servos are detached.")
    pwmCtl.run()


# ---- 3. Data viewing

def _view_accel(link):
    print("Accelerometer (Ctrl+C to return)")
    while True:
        g = link.floats("ACCEL")
        if g is None or len(g) != 3 or g == [0.0, 0.0, 0.0]:
            line = f"no data: {link.diagnose()}"
        else:
            gx, gy, gz = g
            roll, pitch = roll_pitch(gx, gy, gz)
            mag = math.sqrt(gx * gx + gy * gy + gz * gz)
            line = f"g=({gx:+.3f},{gy:+.3f},{gz:+.3f}) |g|={mag:.3f} roll={roll:+6.2f} pitch={pitch:+6.2f}"
        print(f"\r{line:<80}", end="", flush=True)
        time.sleep(DATA_INTERVAL_S)


def _view_lidar(link):
    print(f"Lidar (override below {constants.LIDAR_OVERRIDE_THRESHOLD_MM} mm, Ctrl+C to return)")
    while True:
        v = link.ask("LIDAR").split()
        if len(v) != 3 or "None" in v:
            line = "no data from MCU yet"
        elif v[2] == "0":
            line = "lidar starting up"
        elif v[0] == "-1" and v[2] == "2":
            line = "no reading: VL53L0X library init failed on SDA/SCL (Wire, 0x29), check wiring/power"
        elif v[0] == "-1":
            line = "no reading: sensor initialised but no measurements arriving"
        else:
            line = f"distance={int(v[0]):5d} mm  override={'true' if v[1] == '1' else 'false'}"
        print(f"\r{line:<80}", end="", flush=True)
        time.sleep(DATA_INTERVAL_S)


def data_view(link):
    choice = input("View 1) accelerometer  2) lidar: ").strip()
    try:
        if choice == "1":
            _view_accel(link)
        elif choice == "2":
            _view_lidar(link)
        else:
            print("Unknown choice.")
    finally:
        print()


# ---- 4. Platform levelling

def _loop_hz(link, prev):
    # MCU loop rate from the loop counter; prev = (count, time) from the last call
    try:
        count = int(link.ask("STATUS").split()[0])
    except (ValueError, IndexError):
        return None, prev
    now = time.monotonic()
    hz = None if prev is None else (count - prev[0]) / (now - prev[1])
    return hz, (count, now)


def platform_levelling(link):
    link.mode(constants.MODE_LEVEL)
    print("Platform levelling (Ctrl+C to return)")
    prev = None
    while True:
        hz, prev = _loop_hz(link, prev)
        g = link.floats("ACCEL")
        level = link.floats("LEVEL")
        if g is None or level is None or len(level) != 5 or g == [0.0, 0.0, 0.0]:
            print(f"[level] no data: {link.diagnose()}", flush=True)
        else:
            roll, pitch = level[0], level[1]
            us = tuple(int(u) for u in level[2:])
            rate = "" if hz is None else f" loop={hz:.0f}Hz"
            print(f"[level] g=({g[0]:.3f},{g[1]:.3f},{g[2]:.3f}) roll={roll:.2f} pitch={pitch:.2f} us={us}{rate}", flush=True)
        time.sleep(LEVEL_PRINT_INTERVAL_S)


# ---- 5. Servo calibration

def _pseudo_inverse(a):
    # a: 2 x n (roll row, pitch row) -> n x 2, A^T (A A^T)^-1
    n = len(a[0])
    m00 = sum(a[0][i] * a[0][i] for i in range(n))
    m01 = sum(a[0][i] * a[1][i] for i in range(n))
    m11 = sum(a[1][i] * a[1][i] for i in range(n))
    det = m00 * m11 - m01 * m01
    if abs(det) < 1e-12:
        return None
    i00, i01, i11 = m11 / det, -m01 / det, m00 / det
    return [[a[0][i] * i00 + a[1][i] * i01, a[0][i] * i01 + a[1][i] * i11] for i in range(n)]


def _write_constants(values):
    text = CONSTANTS_H.read_text(encoding="utf-8")
    for name, value in values.items():
        pattern = re.compile(rf"(constexpr\s+float\s+{name}\s*=\s*)-?\d+(?:\.\d+)?f?(\s*;)")
        text, count = pattern.subn(lambda m: f"{m.group(1)}{value:.2f}f{m.group(2)}", text)
        if count != 1:
            raise RuntimeError(f"{name} not found exactly once in {CONSTANTS_H}")
    with open(CONSTANTS_H, "w", encoding="utf-8", newline="") as f:
        f.write(text)


def servo_calibration(link):
    servos = constants.PLATFORM_SERVO_COUNT
    steps = 1 + 3 * servos
    d = constants.CAL_STEP_US
    input("Servo calibration: put the car on a flat surface and keep it still. Enter to start, Ctrl+C to cancel.")
    link.mode(constants.MODE_SERVO_CAL)

    started = time.monotonic()
    running = False
    while True:
        time.sleep(CAL_POLL_S)
        try:
            step = int(link.ask("CALSTEP"))
        except ValueError:
            step = None
        if step is not None and step < steps:
            running = True
        if running and step is not None and step >= steps:
            break
        if not running and time.monotonic() - started > CAL_START_TIMEOUT_S:
            print("\nCalibration did not start on the MCU (was the new sketch uploaded?).")
            return
        print(f"\r[cal] running step {step}/{steps}   ", end="", flush=True)
    print(f"\r[cal] finished {steps}/{steps}        ")

    v = link.floats("CALVALUES")
    if v is None or len(v) != 2 + 4 * servos:
        print("[cal] could not read results from the MCU.")
        return

    print(f"[cal] level reference: roll={v[0]:.2f} pitch={v[1]:.2f}")
    response = [[], []]  # deg/us: roll row, pitch row
    for s in range(servos):
        b = 2 + 4 * s
        pr, pp, mr, mp = v[b:b + 4]
        dr, dp = (pr - mr) / (2 * d), (pp - mp) / (2 * d)
        response[0].append(dr)
        response[1].append(dp)
        print(f"[cal] servo {s}: +{d}us roll={pr:.2f} pitch={pp:.2f} | -{d}us roll={mr:.2f} pitch={mp:.2f} "
              f"| deg/us roll={dr:+.4f} pitch={dp:+.4f}")

    inv = _pseudo_inverse(response)
    if inv is None:
        print("[cal] servo response is degenerate (servos not moving the platform?), nothing changed.")
        return

    new = {"LEVEL_ROLL_ZERO_DEG": v[0], "LEVEL_PITCH_ZERO_DEG": v[1]}
    for s in range(servos):
        new[f"LEVEL_INV_ROLL_{s}"] = inv[s][0]
        new[f"LEVEL_INV_PITCH_{s}"] = inv[s][1]

    print(f"\n{'constant':<24}{'current':>10}{'new':>10}")
    for name in LEVEL_PARAM_NAMES:
        print(f"{name:<24}{getattr(constants, name):>10.2f}{new[name]:>10.2f}")

    if input("\nApply? [y/N]: ").strip().lower() != "y":
        print("Nothing changed.")
        return

    _write_constants(new)
    for i, name in enumerate(LEVEL_PARAM_NAMES):
        link.ask(f"SET {i} {round(new[name], 2)}")
    link.ask("RELOAD")
    importlib.reload(constants)
    print(f"Applied to the MCU and written to {CONSTANTS_H}")
    print("Copy it back to the PC repo, e.g. from the folder above Self-Levelling-Car-Final:")
    print('  scp arduino@<board-ip>:~/ArduinoApps/rc-car/sketch/constants.h "Self-Levelling-Car-Final/sketch/constants.h"')


# ---- 6. Camera view

def camera_view(link):
    import cameraView

    link.mode(constants.MODE_IDLE)  # view only: the car does not act on detections
    print("Camera view (view only, the car will not move).")
    cameraView.run()


# ---- menu loop

OPTIONS = [
    ("Driving", drive),
    ("PWM calibration", pwm_calibration),
    ("Data viewing", data_view),
    ("Platform levelling", platform_levelling),
    ("Servo calibration", servo_calibration),
    ("Camera view", camera_view),
]


def menu():
    try:
        link = Link()
    except OSError:
        print(f"Service not reachable on port {constants.CONTROL_PORT}. Start the app first:")
        print("  arduino-app-cli app start ~/ArduinoApps/rc-car")
        return
    link.mode(constants.MODE_IDLE)

    try:
        while True:
            print("\n==== RC car ====")
            for i, (name, _) in enumerate(OPTIONS, 1):
                print(f"{i}) {name}")
            print("0) Exit")
            choice = input("> ").strip()
            if choice in ("0", "q"):
                break
            if not (choice.isdigit() and 1 <= int(choice) <= len(OPTIONS)):
                continue
            name, fn = OPTIONS[int(choice) - 1]
            print(f"\n-- {name} (Ctrl+C to return to menu) --")
            try:
                fn(link)
            except KeyboardInterrupt:
                print()
            except Exception as e:  # one broken option must not take the whole menu down
                print(f"\n{name} failed: {type(e).__name__}: {e}")
            finally:
                link.mode(constants.MODE_IDLE)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        link.mode(constants.MODE_IDLE)
        link.close()


if "--service" in sys.argv or ("--menu" not in sys.argv and not sys.stdin.isatty()):
    service()
else:
    menu()
