import time
import random
import threading
import argparse

# Attempt to import pyttsx3 for real Text-To-Speech
# Fallback to console simulation if not installed
TTS_AVAILABLE = False
try:
    import pyttsx3
    TTS_AVAILABLE = True
except ImportError:
    pass

# Neutral Home Positions
HOME_MG996R_L = 0.0
HOME_MG996R_R = 180.0
HOME_SG90 = 92.0  # Must match motor_controller_computer / ESP32 home
SG90_MIN_MOVE_DEG = 2
SG90_MAX_GESTURE_STEP = 12.0  # Max relative step per talk tick (continuous sweep)
MG_MAX_GESTURE_STEP = 10.0  # Max change per talk step (voice-agent caps servo steps)
TTS_RATE_WPM = 170

class TalkingHandController:
    def __init__(self, update_interval=0.4, on_angles_callback=None):
        self.update_interval = update_interval
        self.on_angles_callback = on_angles_callback
        self.is_speaking = False
        self.gesture_thread = None
        self.lock = threading.Lock()
        self._tts_available = TTS_AVAILABLE
        self._last_left_mg = HOME_MG996R_L
        self._last_right_mg = HOME_MG996R_R
        self._last_left_sg = HOME_SG90
        self._last_right_sg = HOME_SG90

        # Initialize pyttsx3 engine if available
    def _init_tts_engine(self):
        engine = pyttsx3.init()
        engine.setProperty("rate", TTS_RATE_WPM)
        return engine

    def _estimate_speech_duration_sec(self, text):
        """Lower bound for gesture duration; covers TTS finishing early."""
        words = max(1, len(text.split()))
        wpm_duration = (words / TTS_RATE_WPM) * 60.0
        sim_duration = words * 0.28  # matches _speak_fallback ~0.2–0.35 s/word
        return max(wpm_duration * 1.1, sim_duration, 1.0)

    def _step_mg(self, previous, lo, hi):
        """Small MG996R delta per gesture (smooth layer finishes the motion)."""
        prev = previous
        delta = random.uniform(-MG_MAX_GESTURE_STEP, MG_MAX_GESTURE_STEP)
        return max(lo, min(hi, prev + delta))

    def _random_sg90(self, previous, lo=55.0, hi=85.0):
        """Relative SG90 step (like _step_mg): small delta from previous, clamped to range."""
        prev = round(previous)
        delta = random.uniform(-SG90_MAX_GESTURE_STEP, SG90_MAX_GESTURE_STEP)
        candidate = round(max(lo, min(hi, prev + delta)))

        if abs(candidate - prev) < SG90_MIN_MOVE_DEG:
            # Nudge by minimum step without a large leap — use room toward range interior
            if prev + SG90_MIN_MOVE_DEG <= hi:
                candidate = round(min(hi, prev + SG90_MIN_MOVE_DEG))
            elif prev - SG90_MIN_MOVE_DEG >= lo:
                candidate = round(max(lo, prev - SG90_MIN_MOVE_DEG))
            else:
                return prev

        if abs(candidate - prev) < SG90_MIN_MOVE_DEG:
            return prev
        return int(candidate)

    def _gesture_loop(self):
        """Background loop — full-range random hand poses while speaking (EMA smooths on PC)."""
        print("\n--- [Robot Speaking] Hand Randomizer Started ---")
        self._last_left_mg = HOME_MG996R_L
        self._last_right_mg = HOME_MG996R_R
        self._last_left_sg = HOME_SG90
        self._last_right_sg = HOME_SG90
        while True:
            with self.lock:
                if not self.is_speaking:
                    break

            left_mg = self._step_mg(self._last_left_mg, 60.0, 90.0)
            right_mg = self._step_mg(self._last_right_mg, 90.0, 120.0)
            left_sg = self._random_sg90(self._last_left_sg)
            right_sg = self._random_sg90(self._last_right_sg)
            self._last_left_mg = left_mg
            self._last_right_mg = right_mg
            self._last_left_sg = left_sg
            self._last_right_sg = right_sg
            
            print(f"[TALKING GESTURE] L_M (MG996R): {left_mg:5.1f}° | L_S (SG90): {left_sg:5.1f}° | "
                  f"R_M (MG996R): {right_mg:5.1f}° | R_S (SG90): {right_sg:5.1f}°")
            
            if self.on_angles_callback:
                try:
                    self.on_angles_callback(left_mg, left_sg, right_mg, right_sg)
                except Exception as e:
                    print(f"[CALLBACK ERROR] Failed to send angles: {e}")
            
            time.sleep(self.update_interval)
            
        # Reset to home positions once speech completes
        print(f"[SILENT HOME]     L_M (MG996R): {HOME_MG996R_L:5.1f}° | L_S (SG90): {HOME_SG90:5.1f}° | "
              f"R_M (MG996R): {HOME_MG996R_R:5.1f}° | R_S (SG90): {HOME_SG90:5.1f}°")
        
        if self.on_angles_callback:
            try:
                self.on_angles_callback(HOME_MG996R_L, HOME_SG90, HOME_MG996R_R, HOME_SG90)
            except Exception as e:
                print(f"[CALLBACK ERROR] Failed to send home angles: {e}")
                
        print("--- [Robot Silent] Hand Randomizer Stopped ---\n")

    def _speak_fallback(self, text):
        """Simulate talking by printing text word-by-word with natural delays."""
        words = text.split()
        print(f"\n[Robot Text Output]: ", end="", flush=True)
        for word in words:
            # Check if speaking was cancelled/stopped externally
            with self.lock:
                if not self.is_speaking:
                    break
            print(word + " ", end="", flush=True)
            # Estimate speaking time: average ~0.25 to 0.35 seconds per word
            time.sleep(max(0.15, random.uniform(0.2, 0.35)))
        print()

    def _run_tts(self, text):
        """Run TTS or console simulation in a worker thread."""
        if self._tts_available:
            engine = None
            try:
                engine = self._init_tts_engine()
                engine.say(text)
                engine.runAndWait()
            except Exception as e:
                print(f"[WARNING] TTS failed: {e}. Using simulation timing.")
                self._speak_fallback(text)
            finally:
                if engine is not None:
                    try:
                        engine.stop()
                    except Exception:
                        pass
        else:
            self._speak_fallback(text)

    def speak(self, text):
        """Speak the text and trigger random hand gestures."""
        if not text.strip():
            return
        
        duration_est = self._estimate_speech_duration_sec(text)
        start_time = time.time()
            
        with self.lock:
            self.is_speaking = True

        # Start the background gesture thread
        self.gesture_thread = threading.Thread(target=self._gesture_loop, daemon=True)
        self.gesture_thread.start()

        tts_thread = threading.Thread(target=self._run_tts, args=(text,), daemon=True)
        tts_thread.start()

        try:
            while tts_thread.is_alive() or (time.time() - start_time) < duration_est:
                time.sleep(0.05)
        finally:
            with self.lock:
                self.is_speaking = False
            
            # Wait for gesture thread to cleanly finish and print the home state
            if self.gesture_thread:
                self.gesture_thread.join()

def main():
    parser = argparse.ArgumentParser(description="Robot Talking Hand Gestures Controller")
    parser.add_argument("--interval", type=float, default=0.4, help="Interval between hand angle changes in seconds")
    parser.add_argument("--text", type=str, help="Single phrase for the robot to say, then exit")
    args = parser.parse_args()

    controller = TalkingHandController(update_interval=args.interval)

    print("=====================================================")
    print("   Robot Speech Hand Gestures Controller (Logic-Only)")
    print("=====================================================")
    if TTS_AVAILABLE and controller._tts_available:
        print("  * Text-To-Speech Engine: ACTIVE (pyttsx3, fresh engine per phrase)")
    else:
        print("  * Text-To-Speech Engine: SIMULATION MODE (No pyttsx3)")
    print(f"  * MG996R range: [{40} - {100}] degrees")
    print(f"  * SG90 range:   [{45} - {90}] degrees")
    print(f"  * Update interval: {args.interval}s")
    print("=====================================================\n")

    # If single text phrase provided via CLI
    if args.text:
        controller.speak(args.text)
        return

    # Interactive CLI Mode
    print("Enter text for the robot to say (type 'q', 'exit', or 'quit' to quit):")
    while True:
        try:
            user_input = input("\nRobot Say > ")
            if user_input.lower() in ["q", "exit", "quit"]:
                print("Exiting. Goodbye!")
                break
            controller.speak(user_input)
        except KeyboardInterrupt:
            print("\nExiting. Goodbye!")
            break

if __name__ == "__main__":
    main()
