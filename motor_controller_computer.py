# pyrefly: ignore [missing-import]
"""
Computer vision motor driver with EMA-smoothed servos and optimized serial.

Serial format (compact): "L_MG,L_SG,R_MG,R_SG\\n"  e.g. "96,70,83,55\\n"
Servo updates run on a fixed 50 Hz thread (decoupled from camera FPS).
"""

import cv2
import time
import threading
import random
from test_webcam_hand_control import ByeGestureController
from talking_hand_controller import TalkingHandController

SERIAL_PORT = "COM4"
BAUD_RATE = 115200  # Plenty for ~20-byte packets @ 50 Hz; not the bottleneck

serial_lock = threading.Lock()
ESP32_BOOT_DELAY_SEC = 2.0
BYE_HOLD_SEC = 10.0

# Smoothing / streaming (patterns from voice-agentv4 robot_eyes servo_worker)
SERVO_RATE_HZ = 50.0
SERIAL_KEEPALIVE_SEC = 4.0
AT_TARGET_DEG = 0.5
HOME_SG90 = 92.0
SG90_INDICES = (1, 3)
MG_INDICES = (0, 2)

# MG996R — rate-limited steps (robot_eyes: SMOOTHING=0.10, MAX_STEP=1.4, DEADZONE=0.22)
MG_SMOOTHING = 0.10
MG_MAX_STEP_DEG = 1.2
MG_DEADZONE_DEG = 0.35
MG_GOAL_BLEND = 0.12          # Soft goal tracking before step limiter

# SG90 — slower steps + ignore tiny goal changes
SG90_SMOOTHING = 0.08
SG90_MAX_STEP_DEG = 0.9
SG90_DEADZONE_DEG = 0.5
SG90_MIN_MOVE_DEG = 2
SG90_GOAL_BLEND = 0.15


def _clamp_angle(angle):
    return max(0.0, min(180.0, angle))


def _clamp_step(error, smoothing, max_step, deadzone):
    """voice-agentv4-style capped servo step toward a target."""
    if abs(error) < deadzone:
        return 0.0
    return max(-max_step, min(max_step, error * smoothing))


def _format_packet(angles):
    """Compact CSV — ~15 bytes vs ~45 for legacy L_M: labels."""
    l_mg, l_sg, r_mg, r_sg = (int(round(a)) for a in angles)
    return f"{l_mg},{l_sg},{r_mg},{r_sg}\n"


class SmoothServoController:
    """
    MG996R / SG90: rate-limited motion (voice-agentv4 robot_eyes servo_worker).
    Goals can jump from talk/bye; output moves in small capped steps at 50 Hz.
    """

    def __init__(self, ser_conn):
        self.ser = ser_conn
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self._goal = [0.0, HOME_SG90, 180.0, HOME_SG90]
        self._target = [0.0, HOME_SG90, 180.0, HOME_SG90]
        self._output = [0.0, HOME_SG90, 180.0, HOME_SG90]
        self._last_sent = None
        self._last_send_time = 0.0

    def start(self):
        self._thread = threading.Thread(target=self._servo_loop, daemon=True, name="servo-50hz")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def at_target(self):
        with self._lock:
            return all(
                abs(self._goal[i] - self._output[i]) < AT_TARGET_DEG for i in range(4)
            )

    def _set_sg90_goal(self, idx, angle, force=False):
        value = round(_clamp_angle(angle))
        current = round(self._output[idx])
        if not force and abs(value - current) < SG90_MIN_MOVE_DEG:
            return
        self._goal[idx] = value
        if force:
            self._target[idx] = float(value)
            self._output[idx] = float(value)

    def set_targets(self, left_mg, left_sg, right_mg, right_sg, force_sg90=False):
        goals = [
            _clamp_angle(left_mg),
            _clamp_angle(left_sg),
            _clamp_angle(right_mg),
            _clamp_angle(right_sg),
        ]
        home_snapshot = None
        with self._lock:
            if force_sg90:
                for i in range(4):
                    v = round(goals[i]) if i in SG90_INDICES else goals[i]
                    self._goal[i] = v
                    self._target[i] = float(v)
                    self._output[i] = float(v)
                home_snapshot = [self._output[i] for i in range(4)]
            else:
                self._goal[0] = goals[0]
                self._goal[2] = goals[2]
                self._set_sg90_goal(1, goals[1], force=False)
                self._set_sg90_goal(3, goals[3], force=False)

        if home_snapshot is not None:
            self._send(home_snapshot)

    def _servo_loop(self):
        interval = 1.0 / SERVO_RATE_HZ
        while not self._stop.is_set():
            t0 = time.perf_counter()
            self._tick()
            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _tick(self):
        with self._lock:
            for i in range(4):
                if i in MG_INDICES:
                    self._target[i] += MG_GOAL_BLEND * (self._goal[i] - self._target[i])
                    err = self._target[i] - self._output[i]
                    self._output[i] += _clamp_step(
                        err, MG_SMOOTHING, MG_MAX_STEP_DEG, MG_DEADZONE_DEG
                    )
                else:
                    self._target[i] += SG90_GOAL_BLEND * (self._goal[i] - self._target[i])
                    err = self._target[i] - self._output[i]
                    self._output[i] += _clamp_step(
                        err, SG90_SMOOTHING, SG90_MAX_STEP_DEG, SG90_DEADZONE_DEG
                    )
            snapshot = [_clamp_angle(v) for v in self._output]

        now = time.time()
        if self._should_send(snapshot, now):
            self._send(snapshot)

    def _should_send(self, angles, now):
        if self._last_sent is None:
            return True
        rounded = tuple(int(round(a)) for a in angles)
        for i in range(4):
            if i in SG90_INDICES:
                if abs(rounded[i] - self._last_sent[i]) >= SG90_MIN_MOVE_DEG:
                    return True
            elif rounded[i] != self._last_sent[i]:
                return True
        return now - self._last_send_time >= SERIAL_KEEPALIVE_SEC

    def _send(self, angles):
        if self.ser is None or not self.ser.is_open:
            return
        packet = _format_packet(angles)
        rounded = tuple(int(round(a)) for a in angles)
        try:
            with serial_lock:
                self.ser.write(packet.encode("ascii"))
            self._last_sent = rounded
            self._last_send_time = time.time()
        except Exception as e:
            print(f"[SERIAL ERROR] {e}")


def start_esp32_reader(ser_conn, stop_event):
    """Reads ESP32 debug lines without holding the write lock."""

    def _reader():
        while not stop_event.is_set():
            if ser_conn is None or not ser_conn.is_open:
                break
            try:
                if ser_conn.in_waiting:
                    raw = ser_conn.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        if line:
                            print(f"[ESP32 RX] {line}")
                else:
                    time.sleep(0.02)
            except Exception as e:
                if not stop_event.is_set():
                    print(f"[ESP32 RX ERROR] {e}")
                break

    thread = threading.Thread(target=_reader, daemon=True, name="esp32-serial-reader")
    thread.start()
    return thread


class ByeWaveSequence:
    def __init__(self, smooth):
        self.smooth = smooth
        self._lock = threading.Lock()
        self._state = "idle"
        self._hold_until = 0.0

    @property
    def busy(self):
        with self._lock:
            return self._state != "idle"

    def start(self, left_mg, left_sg, right_mg, right_sg, active_side):
        with self._lock:
            if self._state != "idle":
                return False
            self._state = "to_wave"
        self.smooth.set_targets(left_mg, left_sg, right_mg, right_sg)
        print(
            f"[MOTOR CONTROLLER] Bye — {active_side} arm wave "
            f"(hold {BYE_HOLD_SEC:.0f}s after arrival)"
        )
        return True

    def tick(self):
        with self._lock:
            state = self._state

        if state == "to_wave" and self.smooth.at_target():
            with self._lock:
                self._state = "hold"
                self._hold_until = time.time() + BYE_HOLD_SEC
            print(f"[MOTOR CONTROLLER] Wave pose reached — holding {BYE_HOLD_SEC:.0f}s.")
        elif state == "hold":
            with self._lock:
                hold_until = self._hold_until
            if time.time() >= hold_until:
                with self._lock:
                    self._state = "to_home"
                print("[MOTOR CONTROLLER] Returning home.")
                self.smooth.set_targets(0.0, HOME_SG90, 180.0, HOME_SG90, force_sg90=True)
        elif state == "to_home" and self.smooth.at_target():
            with self._lock:
                self._state = "idle"
            print("[MOTOR CONTROLLER] Home reached.")


def get_bye_callback(smooth, bye_seq):
    def handle_bye(physical_side, intensity):
        active_side = random.choice(["LEFT", "RIGHT"])
        mg_l_wave = random.uniform(150.0, 170.0)
        mg_r_wave = random.uniform(10.0, 30.0)
        sg_wave = random.uniform(80.0, 100.0)
        left_mg = mg_l_wave if active_side == "LEFT" else 0.0
        left_sg = sg_wave if active_side == "LEFT" else HOME_SG90
        right_mg = mg_r_wave if active_side == "RIGHT" else 180.0
        right_sg = sg_wave if active_side == "RIGHT" else HOME_SG90
        if not bye_seq.start(left_mg, left_sg, right_mg, right_sg, active_side):
            print("[MOTOR CONTROLLER] Bye ignored — sequence in progress.")

    return handle_bye


def get_talk_callback(smooth, bye_seq):
    def handle_talk_angles(left_mg, left_sg, right_mg, right_sg):
        if bye_seq.busy:
            return
        force_sg90 = round(left_sg) == round(HOME_SG90) and round(right_sg) == round(HOME_SG90)
        smooth.set_targets(left_mg, left_sg, right_mg, right_sg, force_sg90=force_sg90)
        print(
            f"[TALK RANDOMIZER] L_M:{left_mg:5.1f} L_S:{left_sg:5.1f} | "
            f"R_M:{right_mg:5.1f} R_S:{right_sg:5.1f}"
        )

    return handle_talk_angles


def main():
    print("\n=======================================================")
    print("      [MOTOR CONTROLLER] EMA + 50 Hz serial stream")
    print("=======================================================")
    print(
        f"  * Servo thread: {SERVO_RATE_HZ:.0f} Hz | "
        f"MG step≤{MG_MAX_STEP_DEG}° SG90 step≤{SG90_MAX_STEP_DEG}°"
    )
    print(f"  * Serial: compact CSV @ {BAUD_RATE} baud on {SERIAL_PORT}")
    print("  * Press ESC or Q to quit.")
    print("=======================================================\n")

    ser = None
    esp32_reader_stop = threading.Event()
    esp32_reader_thread = None
    try:
        import serial

        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.05)
        print(f"[SERIAL] Connected {SERIAL_PORT} @ {BAUD_RATE}")
        print(f"[SERIAL] Boot wait {ESP32_BOOT_DELAY_SEC:.0f}s...")
        time.sleep(ESP32_BOOT_DELAY_SEC)
        ser.reset_input_buffer()
        esp32_reader_thread = start_esp32_reader(ser, esp32_reader_stop)
    except Exception as e:
        print(f"[SERIAL WARNING] {e}")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Webcam not available.")
        if ser is not None and ser.is_open:
            ser.close()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    smooth = SmoothServoController(ser)
    if ser is not None and ser.is_open:
        smooth.start()

    bye_seq = ByeWaveSequence(smooth)
    controller = ByeGestureController(on_bye_callback=get_bye_callback(smooth, bye_seq))
    talk_controller = TalkingHandController(
        update_interval=1.4,
        on_angles_callback=get_talk_callback(smooth, bye_seq),
    )

    def console_input():
        print("\n[TALK CONSOLE] Type text + Enter ('q' to exit console).")
        while True:
            try:
                txt = input()
                if txt.lower() in ("q", "exit", "quit"):
                    break
                if txt.strip():
                    talk_controller.speak(txt)
            except (KeyboardInterrupt, EOFError):
                break

    threading.Thread(target=console_input, daemon=True).start()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.03)
                continue
            processed = controller.process_frame(frame)
            bye_seq.tick()
            cv2.imshow("AI Bye Controller - Computer Motor Driver", processed)
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
    finally:
        print("\nShutting down...")
        cap.release()
        controller.close()
        cv2.destroyAllWindows()
        smooth.stop()
        esp32_reader_stop.set()
        if esp32_reader_thread is not None:
            esp32_reader_thread.join(timeout=1.0)
        if ser is not None and ser.is_open:
            try:
                smooth.set_targets(0.0, HOME_SG90, 180.0, HOME_SG90, force_sg90=True)
                time.sleep(0.8)
                ser.close()
            except Exception as e:
                print(f"[SERIAL CLEANUP ERROR] {e}")
        print("[SUCCESS] Goodbye!")


if __name__ == "__main__":
    main()
