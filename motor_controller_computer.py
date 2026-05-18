# pyrefly: ignore [missing-import]
import cv2
import time
from test_webcam_hand_control import ByeGestureController

import random

def handle_bye(physical_side, intensity):
    """
    Event listener triggered when ByeGestureController detects a hand waving above shoulder level.
    """
    # Start waving: MG996R goes to randomly 155-165, and SG90 goes to randomly 85-95
    mg996r_angle = random.uniform(155.0, 165.0)
    sg90_angle = random.uniform(85.0, 95.0)
    print(f"[MOTOR CONTROLLER] say bye (Hand: {physical_side.upper()} | Intensity: {intensity:.2f} | Pitch (MG996R): {mg996r_angle:.1f} | Yaw (SG90): {sg90_angle:.1f})")

def main():
    # Phase 1: Establish starting angles
    mg996r_angle = 0.0
    sg90_angle = 90.0

    print("\n=======================================================")
    print("      [MOTOR CONTROLLER] Driver Initialized")
    print("=======================================================")
    print(f"  * Starting Postures established:")
    print(f"    - MG996R Shoulder Pitch (X-axis): {mg996r_angle:.1f} degrees (downward)")
    print(f"    - SG90 Elbow Yaw (Z-axis): {sg90_angle:.1f} degrees (forward)")
    print("  * Webcam opened & managed by motor_controller_computer.py")
    print("  * Vision intelligence imported from test_webcam_hand_control.py")
    print("  * Prints 'say bye' statement when waving above shoulder.")
    print("  * Press 'ESC' or 'Q' to quit.")
    print("=======================================================\n")

    # Open Camera index 0
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot access the webcam (index 0).")
        return

    # Set frame dimension constraints
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Initialize the vision engine, registering our handle_bye print listener
    controller = ByeGestureController(on_bye_callback=handle_bye)

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
        print("[SUCCESS] Shutdown completed. Goodbye!")

if __name__ == "__main__":
    main()
