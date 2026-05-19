# pyrefly: ignore [missing-import]
import cv2
import time
from test_webcam_hand_control import ByeGestureController

import random

# Serial Communication Constants
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

def get_bye_callback(ser_conn):
    """
    Returns a custom handle_bye listener bound to the active serial connection context.
    """
    def handle_bye(physical_side, intensity):
        # 1. Randomly select which physical arm waves
        active_side = random.choice(["LEFT", "RIGHT"])
        
        # Start waving: MG996R goes to randomly 150-170, and SG90 goes to randomly 70-110 (wider visible range)
        mg_wave = random.uniform(150.0, 170.0)
        sg_wave = random.uniform(00.0, 40.0)
        
        # 2. Assign positions (waving side gets waving angles, non-waving side stays at home 0 and 90)
        left_mg  = mg_wave if active_side == "LEFT" else 0.0
        left_sg  = sg_wave if active_side == "LEFT" else 90.0
        right_mg = mg_wave if active_side == "RIGHT" else 0.0
        right_sg = sg_wave if active_side == "RIGHT" else 90.0
        
        print(f"[MOTOR CONTROLLER] Bye detected! Active physical arm: {active_side} | "
              f"Left Arm: (Pitch MG996R: {left_mg:.1f}, Yaw SG90: {left_sg:.1f}) | "
              f"Right Arm: (Pitch MG996R: {right_mg:.1f}, Yaw SG90: {right_sg:.1f})")
        
        # Write to serial if active
        if ser_conn is not None and ser_conn.is_open:
            command = f"L_M:{left_mg:.1f},L_S:{left_sg:.1f},R_M:{right_mg:.1f},R_S:{right_sg:.1f}\n"
            try:
                ser_conn.write(command.encode('utf-8'))
                print(f"[SERIAL SENT] {command.strip()}")
            except Exception as e:
                print(f"[SERIAL ERROR] Failed to write command: {e}")
                
    return handle_bye

def main():
    # Phase 1: Establish starting angles
    left_mg = 0.0
    left_sg = 90.0
    right_mg = 0.0
    right_sg = 90.0

    print("\n=======================================================")
    print("      [MOTOR CONTROLLER] Driver Initialized")
    print("=======================================================")
    print(f"  * Starting Postures established for Dual-Arm setup:")
    print(f"    - Left Arm  -> MG996R Pitch: {left_mg:.1f} | SG90 Yaw: {left_sg:.1f}")
    print(f"    - Right Arm -> MG996R Pitch: {right_mg:.1f} | SG90 Yaw: {right_sg:.1f}")
    print("  * Webcam opened & managed by motor_controller_computer.py")
    print("  * Vision intelligence imported from test_webcam_hand_control.py")
    print("  * Randomly selects Left or Right physical arm to wave when bye is detected.")
    print("  * Press 'ESC' or 'Q' to quit.")
    print("=======================================================\n")

    # Initialize Serial connection with ESP32
    ser = None
    try:
        import serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[SERIAL] Connected to ESP32 on {SERIAL_PORT} @ {BAUD_RATE} baud.")
    except Exception as e:
        print(f"[SERIAL WARNING] Could not connect to {SERIAL_PORT}: {e}")
        print("[SERIAL] Operating in Simulation / Visual-Only mode.")

    # Open Camera index 0
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot access the webcam (index 0).")
        if ser is not None and ser.is_open:
            ser.close()
        return

    # Set frame dimension constraints
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Initialize the vision engine, registering our serial-bound callback
    bye_callback = get_bye_callback(ser)
    controller = ByeGestureController(on_bye_callback=bye_callback)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARNING] Frame acquisition failed. Retrying...")
                time.sleep(0.03)
                continue

            # Process the frame using the vision engine logic library
            processed = controller.process_frame(frame)

            # Display the interactive visualizer HUD in a window
            cv2.imshow("AI Bye Controller - Computer Motor Driver", processed)

            # Exit check: ESC or 'q'
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
    finally:
        print("\nShutting down driver stream...")
        cap.release()
        controller.close()
        cv2.destroyAllWindows()
        
        # Cleanly release serial resources
        if ser is not None and ser.is_open:
            try:
                # Optionally command servos back to their home position (downward & forward)
                home_command = "L_M:0.0,L_S:90.0,R_M:0.0,R_S:90.0\n"
                ser.write(home_command.encode('utf-8'))
                ser.close()
                print("[SERIAL] Sent home posture to both arms and closed connection cleanly.")
            except Exception as e:
                print(f"[SERIAL CLEANUP ERROR] {e}")
                
        print("[SUCCESS] Shutdown completed. Goodbye!")

if __name__ == "__main__":
    main()
