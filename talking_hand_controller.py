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
HOME_MG996R = 0.0
HOME_SG90 = 90.0

class TalkingHandController:
    def __init__(self, update_interval=0.4, on_angles_callback=None):
        self.update_interval = update_interval
        self.on_angles_callback = on_angles_callback
        self.is_speaking = False
        self.gesture_thread = None
        self.lock = threading.Lock()

        # Initialize pyttsx3 engine if available
        if TTS_AVAILABLE:
            try:
                self.engine = pyttsx3.init()
                # Adjust speech rate (default is usually ~200, 160-180 sounds more natural)
                self.engine.setProperty('rate', 170)
            except Exception as e:
                print(f"[WARNING] TTS engine initialization failed: {e}. Falling back to simulation.")
                self.engine = None
        else:
            self.engine = None

    def _gesture_loop(self):
        """Background loop — full-range random hand poses while speaking (EMA smooths on PC)."""
        print("\n--- [Robot Speaking] Hand Randomizer Started ---")
        while True:
            with self.lock:
                if not self.is_speaking:
                    break

            # MG996R: 40–100° | SG90: 45–90° (independent random each step)
            left_mg = random.uniform(40.0, 100.0)
            right_mg = random.uniform(40.0, 100.0)
            left_sg = random.uniform(45.0, 90.0)
            right_sg = random.uniform(45.0, 90.0)
            
            print(f"[TALKING GESTURE] L_M (MG996R): {left_mg:5.1f}° | L_S (SG90): {left_sg:5.1f}° | "
                  f"R_M (MG996R): {right_mg:5.1f}° | R_S (SG90): {right_sg:5.1f}°")
            
            if self.on_angles_callback:
                try:
                    self.on_angles_callback(left_mg, left_sg, right_mg, right_sg)
                except Exception as e:
                    print(f"[CALLBACK ERROR] Failed to send angles: {e}")
            
            time.sleep(self.update_interval)
            
        # Reset to home positions once speech completes
        print(f"[SILENT HOME]     L_M (MG996R): {HOME_MG996R:5.1f}° | L_S (SG90): {HOME_SG90:5.1f}° | "
              f"R_M (MG996R): {HOME_MG996R:5.1f}° | R_S (SG90): {HOME_SG90:5.1f}°")
        
        if self.on_angles_callback:
            try:
                self.on_angles_callback(HOME_MG996R, HOME_SG90, HOME_MG996R, HOME_SG90)
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

    def speak(self, text):
        """Speak the text and trigger random hand gestures."""
        if not text.strip():
            return
            
        with self.lock:
            self.is_speaking = True

        # Start the background gesture thread
        self.gesture_thread = threading.Thread(target=self._gesture_loop, daemon=True)
        self.gesture_thread.start()

        try:
            if self.engine:
                # Active TTS mode
                self.engine.say(text)
                self.engine.runAndWait()
            else:
                # Simulation mode
                self._speak_fallback(text)
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
    if TTS_AVAILABLE and controller.engine:
        print("  * Text-To-Speech Engine: ACTIVE (pyttsx3)")
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
