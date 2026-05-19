# pyrefly: ignore [missing-import]
import cv2
# pyrefly: ignore [missing-import]
import mediapipe as mp
# pyrefly: ignore [missing-import]
import numpy as np
# pyrefly: ignore [missing-import]
import collections
import time
import math

class ByeGestureController:
    def __init__(self, camera_index=0, on_bye_callback=None):
        self.camera_index = camera_index
        self.on_bye_callback = on_bye_callback
        
        # Wave Detection Constants
        self.WAVE_HISTORY_LEN = 25       # Rolling window size for wave history (approx. 1 second at 25-30fps)
        self.WAVE_DEAD_ZONE_PX = 10      # Ignores minor hand/sensor jitter below this threshold
        self.WAVE_MIN_REVERSALS = 3      # Number of directional shifts required to confirm a wave
        self.WAVE_MIN_AMPLITUDE_PX = 40  # Minimum sweep size (in pixels) to register a wave
        self.WAVE_TRIGGER_SWINGS = 8     # Required swings to trigger the bye event callback
        self.WAVE_COOLDOWN_SEC = 16.0    # Cooldown duration in seconds (aligned with ESP32's 15s watchdog)
        self.cooldown_until = 0.0        # Timestamp when the cooldown expires
        self.announcement_end_time = 0.0 # Timestamp when center banner ends
        self.announcement_hand = ""      # Hand that triggered the announcement
        
        # State tracking for both hands
        self.hand_states = {
            "Left": {
                "x_history": collections.deque(maxlen=self.WAVE_HISTORY_LEN),
                "y_history": collections.deque(maxlen=self.WAVE_HISTORY_LEN),
                "last_seen": 0.0,
                "is_waving": False,
                "above_shoulder": False,
                "is_frontside": True,
                "wave_intensity": 0.0,
                "reversals": 0,
                "amplitude": 0.0,
                "last_log_time": 0.0,
                "colors": {
                    "idle_joint": (0, 100, 255),      # Neon Orange
                    "idle_line": (0, 165, 255),
                    "active_joint": (255, 255, 0),    # Cyan/Yellow
                    "active_line": (255, 200, 0)
                }
            },
            "Right": {
                "x_history": collections.deque(maxlen=self.WAVE_HISTORY_LEN),
                "y_history": collections.deque(maxlen=self.WAVE_HISTORY_LEN),
                "last_seen": 0.0,
                "is_waving": False,
                "above_shoulder": False,
                "is_frontside": True,
                "wave_intensity": 0.0,
                "reversals": 0,
                "amplitude": 0.0,
                "last_log_time": 0.0,
                "colors": {
                    "idle_joint": (0, 100, 255),      # Neon Orange
                    "idle_line": (0, 165, 255),
                    "active_joint": (255, 255, 0),    # Cyan/Yellow
                    "active_line": (255, 200, 0)
                }
            }
        }
        
        # MediaPipe Hands Setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,                   # Track both left and right hands
            model_complexity=1,                # 1 = balanced speed/accuracy
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        
        # MediaPipe Pose Setup
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            model_complexity=0,                # 0 = extremely fast for real-time CPU/webcam execution
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Performance variables
        self.prev_time = time.time()
        self.fps = 0.0

    def detect_waving(self, history):
        """
        Detects waving motion by counting peak/valley direction reversals 
        in a coordinate history window with noise filtering (dead zone).
        """
        if len(history) < 6:
            return 0, 0.0

        reversals = 0
        anchor = history[0]
        direction = 0  # +1 = moving right/down, -1 = moving left/up
        peak_vals = [history[0]]

        for pos in history[1:]:
            diff = pos - anchor
            if abs(diff) < self.WAVE_DEAD_ZONE_PX:
                continue  # skip noise within the dead zone
            
            new_dir = 1 if diff > 0 else -1
            if direction != 0 and new_dir != direction:
                reversals += 1
                peak_vals.append(anchor)
            direction = new_dir
            anchor = pos

        amplitude = max(peak_vals) - min(peak_vals) if peak_vals else 0.0
        return reversals, amplitude

    def process_frame(self, frame):
        """
        Process the frame to perform:
        1. Camera mirroring.
        2. MediaPipe hand & pose tracking.
        3. Corrected physical handedness tracking.
        4. Shoulder height restriction checks.
        5. Visual interaction zone rendering with dynamic level lines.
        6. Custom HUD overlay rendering (glassmorphism details, trails, gauges).
        """
        # 1. Horizontal mirror flip for natural user interaction
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Run MediaPipe models
        results_hands = self.hands.process(rgb_frame)
        results_pose = self.pose.process(rgb_frame)
        
        now = time.time()
        
        # Reset tracking data for hands that haven't been seen recently (timeout of 0.4 seconds)
        for side in ["Left", "Right"]:
            if now - self.hand_states[side]["last_seen"] > 0.4:
                self.hand_states[side]["x_history"].clear()
                self.hand_states[side]["y_history"].clear()
                self.hand_states[side]["is_waving"] = False
                self.hand_states[side]["above_shoulder"] = False
                self.hand_states[side]["is_frontside"] = True
                self.hand_states[side]["wave_intensity"] = 0.0
                self.hand_states[side]["reversals"] = 0
                self.hand_states[side]["amplitude"] = 0.0

        # Extract shoulders from MediaPipe Pose
        left_shoulder_y = None
        right_shoulder_y = None
        
        if results_pose.pose_landmarks:
            pose_lms = results_pose.pose_landmarks.landmark
            # Left Shoulder (Index 11) - Physical Left (shown on the right half of mirrored screen)
            ls = pose_lms[11]
            if ls.visibility > 0.5:
                left_shoulder_y = int(ls.y * h)
                
            # Right Shoulder (Index 12) - Physical Right (shown on the left half of mirrored screen)
            rs = pose_lms[12]
            if rs.visibility > 0.5:
                right_shoulder_y = int(rs.y * h)

        active_waving_hands = []

        # Process detected hands
        if results_hands.multi_hand_landmarks and results_hands.multi_handedness:
            for hand_landmarks, handedness in zip(results_hands.multi_hand_landmarks, results_hands.multi_handedness):
                # 2. Handedness Correction: MediaPipe classification is inverted on mirrored webcams
                mp_label = handedness.classification[0].label  # "Left" or "Right"
                physical_side = "Left" if mp_label == "Right" else "Right"
                
                state = self.hand_states[physical_side]
                state["last_seen"] = now
                
                # 2.5 Palm facing orientation check (Frontside vs Backside Knuckles)
                # We calculate the 2D cross-product of palm vectors from Wrist (0) to Index MCP (5) and Pinky MCP (17)
                w_lm = hand_landmarks.landmark[0]
                i_lm = hand_landmarks.landmark[5]
                p_lm = hand_landmarks.landmark[17]
                
                v1_x = i_lm.x - w_lm.x
                v1_y = i_lm.y - w_lm.y
                v2_x = p_lm.x - w_lm.x
                v2_y = p_lm.y - w_lm.y
                
                cp = (v1_x * v2_y) - (v1_y * v2_x)
                if mp_label == "Left":
                    is_frontside = cp < 0.0
                else:
                    is_frontside = cp > 0.0
                    
                state["is_frontside"] = is_frontside
                
                # Get screen space coordinates for landmarks (scale by frame width/height)
                lm_pts = []
                for lm in hand_landmarks.landmark:
                    lm_pts.append((int(lm.x * w), int(lm.y * h)))
                
                # 3. Use Palm Center (average of Wrist 0, Index MCP 5, and Pinky MCP 17) to track movement
                wrist = lm_pts[0]
                index_mcp = lm_pts[5]
                pinky_mcp = lm_pts[17]
                palm_x = (wrist[0] + index_mcp[0] + pinky_mcp[0]) // 3
                palm_y = (wrist[1] + index_mcp[1] + pinky_mcp[1]) // 3
                
                # Append coordinates to sliding history ONLY if palm is facing the camera
                if is_frontside:
                    state["x_history"].append(palm_x)
                    state["y_history"].append(palm_y)
                else:
                    state["x_history"].clear()
                    state["y_history"].clear()
                
                # 4. Shoulder Height Restriction: Get hand-specific shoulder level line
                shoulder_y = left_shoulder_y if physical_side == "Left" else right_shoulder_y
                
                # If target shoulder is not visible, fallback to the other shoulder or a frame ratio threshold
                if shoulder_y is None:
                    other_shoulder = right_shoulder_y if physical_side == "Left" else left_shoulder_y
                    if other_shoulder is not None:
                        shoulder_y = other_shoulder
                    else:
                        shoulder_y = int(0.45 * h)  # Fallback: hand in the upper 45% of the frame
                
                # In image coordinates, smaller Y values represent positions higher on the screen
                is_above_shoulder = palm_y < shoulder_y
                state["above_shoulder"] = is_above_shoulder
                
                # 5. Wave detection calculations
                rev_x, amp_x = self.detect_waving(list(state["x_history"]))
                rev_y, amp_y = self.detect_waving(list(state["y_history"]))
                
                # Side-to-side waving (X) and up-down waving (Y) combined: X is primary, but we accept both
                best_rev = max(rev_x, rev_y)
                best_amp = max(amp_x, amp_y)
                
                state["reversals"] = best_rev
                state["amplitude"] = best_amp
                
                # Check waving classification trigger (MUST BE ABOVE SHOULDER)
                is_waving = (best_rev >= self.WAVE_MIN_REVERSALS and 
                             best_amp >= self.WAVE_MIN_AMPLITUDE_PX and 
                             is_above_shoulder)
                state["is_waving"] = is_waving
                
                # Calculate wave intensity gauge score (0.0 to 1.0)
                # Drains to 0 immediately if hand falls below shoulder line
                if is_above_shoulder:
                    intensity = min(1.0, best_amp / 160.0) * min(1.0, best_rev / 5.0)
                else:
                    intensity = 0.0
                state["wave_intensity"] = intensity
                
                # Trigger the official bye event when swings >= 8, hand is above shoulder, and not on cooldown
                bye_event_triggered = (best_rev >= self.WAVE_TRIGGER_SWINGS and 
                                       is_above_shoulder and 
                                       now > self.cooldown_until)
                                       
                if bye_event_triggered:
                    # Set the 30-second cooldown lock
                    self.cooldown_until = now + self.WAVE_COOLDOWN_SEC
                    
                    # Set the screen center announcement for 4.0 seconds
                    self.announcement_end_time = now + 4.0
                    self.announcement_hand = physical_side
                    
                    # Fire the callback (sends data)
                    if self.on_bye_callback:
                        self.on_bye_callback(physical_side, intensity)
                        
                    # Print to console exactly once when triggered
                    print(f"[BYE TRIGGERED] {physical_side.upper()} HAND waving detected! "
                          f"(Swings: {best_rev} >= 8 | Intensity: {intensity:.2f}) - Lock-out for 30s initiated.")
                          
                if is_waving:
                    active_waving_hands.append(physical_side)

                # 6. Draw Glowing Motion Trails
                self.draw_motion_trail(frame, list(state["x_history"]), list(state["y_history"]), is_waving)

                # Determine skeleton overlay colors
                colors = state["colors"]
                joint_col = colors["active_joint"] if is_waving else colors["idle_joint"]
                line_col = colors["active_line"] if is_waving else colors["idle_line"]
                
                # Draw custom premium hand skeleton lines
                for connection in self.mp_hands.HAND_CONNECTIONS:
                    pt1 = lm_pts[connection[0]]
                    pt2 = lm_pts[connection[1]]
                    cv2.line(frame, pt1, pt2, line_col, 2, cv2.LINE_AA)
                
                # Draw custom joint points
                for pt in lm_pts:
                    cv2.circle(frame, pt, 4, joint_col, -1, cv2.LINE_AA)
                    cv2.circle(frame, pt, 5, (255, 255, 255), 1, cv2.LINE_AA) # white ring

        # 7. Render dynamic shoulder level lines and targets
        if results_pose.pose_landmarks:
            pose_lms = results_pose.pose_landmarks.landmark
            overlay = frame.copy()
            
            # Left Shoulder (Landmark 11) - Mirrored Right side of frame
            ls = pose_lms[11]
            if ls.visibility > 0.5:
                ls_x, ls_y = int(ls.x * w), int(ls.y * h)
                col = (255, 255, 0) if self.hand_states["Left"]["above_shoulder"] else (0, 100, 255) # Cyan vs Orange
                # Draw dashed/level line across left hand zone (right half of frame)
                cv2.line(overlay, (w // 2, ls_y), (w, ls_y), col, 2, cv2.LINE_AA)
                cv2.circle(overlay, (ls_x, ls_y), 7, col, -1, cv2.LINE_AA)
                cv2.circle(overlay, (ls_x, ls_y), 11, (255, 255, 255), 1, cv2.LINE_AA)
                
            # Right Shoulder (Landmark 12) - Mirrored Left side of frame
            rs = pose_lms[12]
            if rs.visibility > 0.5:
                rs_x, rs_y = int(rs.x * w), int(rs.y * h)
                col = (255, 255, 0) if self.hand_states["Right"]["above_shoulder"] else (0, 100, 255) # Cyan vs Orange
                # Draw dashed/level line across right hand zone (left half of frame)
                cv2.line(overlay, (0, rs_y), (w // 2, rs_y), col, 2, cv2.LINE_AA)
                cv2.circle(overlay, (rs_x, rs_y), 7, col, -1, cv2.LINE_AA)
                cv2.circle(overlay, (rs_x, rs_y), 11, (255, 255, 255), 1, cv2.LINE_AA)
                
            # Draw center dividing dashed line to show left/right zones
            cv2.line(overlay, (w // 2, 80), (w // 2, h), (120, 120, 120), 1, cv2.LINE_AA)
            
            # Blend targets and level lines
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        # Calculate FPS
        fps_now = time.time()
        self.fps = 1.0 / max(fps_now - self.prev_time, 1e-6)
        self.prev_time = fps_now
        
        # 8. Render Premium Glassmorphic HUD Overlays
        frame = self.render_hud(frame, active_waving_hands)
        
        return frame

    def draw_motion_trail(self, frame, x_history, y_history, is_waving):
        """
        Draws a glowing, fading trail of the palm center's path.
        """
        points = list(zip(x_history, y_history))
        num_pts = len(points)
        if num_pts < 2:
            return
            
        overlay = frame.copy()
        
        # Pick vibrant trail color: neon cyan for waving, neon green/orange for idle
        trail_color = (255, 255, 0) if is_waving else (0, 200, 100) # Cyan vs Mint Green
        
        for i in range(num_pts - 1):
            alpha = (i + 1) / num_pts
            radius = int(2 + 6 * alpha)
            pt1 = (int(points[i][0]), int(points[i][1]))
            pt2 = (int(points[i+1][0]), int(points[i+1][1]))
            
            # Semi-transparent trail line and point glow
            cv2.line(overlay, pt1, pt2, trail_color, int(radius / 2), cv2.LINE_AA)
            cv2.circle(overlay, pt2, radius, trail_color, -1, cv2.LINE_AA)
            
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

    def render_hud(self, frame, active_waving_hands):
        """
        Draws glassmorphism HUD overlays, top header bar, intensity gauges, 
        and giant center screens for active 'Bye' detections.
        """
        now = time.time()
        h, w, _ = frame.shape
        overlay = frame.copy()
        
        # --- A. Glassmorphism Top Panel ---
        # Dark, semi-transparent top header bar
        cv2.rectangle(overlay, (0, 0), (w, 80), (15, 15, 15), -1)
        # Deep orange bottom border line for the header
        cv2.line(overlay, (0, 80), (w, 80), (255, 100, 0), 2, cv2.LINE_AA) 
        
        # Add glassmorphism blend
        cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
        
        # Header Text
        cv2.putText(frame, "AI BYE CONTROLLER", (20, 32), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "MediaPipe Hand & Pose Height Restriction", (20, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        
        # FPS Status
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (w - 120, 45), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 150), 2, cv2.LINE_AA)
        
        # --- B. Waving Status Side Panels (Bottom Corners) ---
        hud_overlay = frame.copy()
        
        # Left Hand Stats Panel (Bottom-Left Corner)
        cv2.rectangle(hud_overlay, (15, h - 135), (245, h - 15), (25, 20, 20), -1)
        cv2.rectangle(hud_overlay, (15, h - 135), (245, h - 15), (255, 165, 0), 1, cv2.LINE_AA) # Orange border
        
        # Right Hand Stats Panel (Bottom-Right Corner)
        cv2.rectangle(hud_overlay, (w - 245, h - 135), (w - 15, h - 15), (25, 20, 20), -1)
        cv2.rectangle(hud_overlay, (w - 245, h - 135), (w - 15, h - 15), (255, 165, 0), 1, cv2.LINE_AA) # Orange border
        
        # Blend the bottom panels
        cv2.addWeighted(hud_overlay, 0.75, frame, 0.25, 0, frame)
        
        # Fill Left Hand Text & Gauge
        left_state = self.hand_states["Left"]
        left_seen = (time.time() - left_state["last_seen"]) < 0.4
        cv2.putText(frame, "LEFT HAND", (25, h - 110), cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        
        if left_seen:
            # Multi-stage status checking
            if now < self.cooldown_until:
                lbl = "COOLDOWN ACTIVE ⏳"
                col = (0, 100, 255) # Orange
            elif not left_state["is_frontside"]:
                lbl = "BACKSIDE [IGNORED]"
                col = (0, 0, 255) # Warning Red
            elif left_state["above_shoulder"]:
                if left_state["reversals"] >= 8:
                    lbl = "BYE DETECTED! 👋"
                    col = (255, 255, 0) # Cyan
                elif left_state["reversals"] >= 3:
                    lbl = f"SWING SPEED: {left_state['reversals']}/8"
                    col = (0, 255, 100) # Mint
                else:
                    lbl = "WAVE TO SAY BYE!"
                    col = (0, 255, 100)
            else:
                lbl = "RAISE HAND ABOVE SHOULDER"
                col = (0, 165, 255)
                
            above_str = "YES" if left_state["above_shoulder"] else "NO"
            above_col = (0, 255, 100) if left_state["above_shoulder"] else (100, 100, 255)
            
            cv2.putText(frame, lbl, (25, h - 88), cv2.FONT_HERSHEY_DUPLEX, 0.45, col, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Above Shoulder: {above_str}", (25, h - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.4, above_col, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Swings: {left_state['reversals']} | Sweep: {left_state['amplitude']:.0f}px", (25, h - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
            # Render left intensity bar
            self.draw_gauge(frame, (25, top_y := h - 30), 210, left_state["wave_intensity"])
        else:
            cv2.putText(frame, "OUT OF FRAME", (25, h - 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1, cv2.LINE_AA)
            self.draw_gauge(frame, (25, h - 30), 210, 0.0)

        # Fill Right Hand Text & Gauge
        right_state = self.hand_states["Right"]
        right_seen = (time.time() - right_state["last_seen"]) < 0.4
        cv2.putText(frame, "RIGHT HAND", (w - 235, h - 110), cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        
        if right_seen:
            # Multi-stage status checking
            if now < self.cooldown_until:
                lbl = "COOLDOWN ACTIVE ⏳"
                col = (0, 100, 255) # Orange
            elif not right_state["is_frontside"]:
                lbl = "BACKSIDE [IGNORED]"
                col = (0, 0, 255) # Warning Red
            elif right_state["above_shoulder"]:
                if right_state["reversals"] >= 8:
                    lbl = "BYE DETECTED! 👋"
                    col = (255, 255, 0) # Cyan
                elif right_state["reversals"] >= 3:
                    lbl = f"SWING SPEED: {right_state['reversals']}/8"
                    col = (0, 255, 100) # Mint
                else:
                    lbl = "WAVE TO SAY BYE!"
                    col = (0, 255, 100)
            else:
                lbl = "RAISE HAND ABOVE SHOULDER"
                col = (0, 165, 255)
                
            above_str = "YES" if right_state["above_shoulder"] else "NO"
            above_col = (0, 255, 100) if right_state["above_shoulder"] else (100, 100, 255)
            
            cv2.putText(frame, lbl, (w - 235, h - 88), cv2.FONT_HERSHEY_DUPLEX, 0.45, col, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Above Shoulder: {above_str}", (w - 235, h - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.4, above_col, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Swings: {right_state['reversals']} | Sweep: {right_state['amplitude']:.0f}px", (w - 235, h - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
            # Render right intensity bar
            self.draw_gauge(frame, (w - 235, h - 30), 210, right_state["wave_intensity"])
        else:
            cv2.putText(frame, "OUT OF FRAME", (w - 235, h - 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1, cv2.LINE_AA)
            self.draw_gauge(frame, (w - 235, h - 30), 210, 0.0)

        # --- C. Central Large Bye Announcement Banner (Flashing for 4 seconds after trigger) ---
        now = time.time()
        if now < self.announcement_end_time:
            center_overlay = frame.copy()
            
            banner_w, banner_h = 420, 80
            bx = (w - banner_w) // 2
            by = h // 2 - 40
            
            # Semi-transparent dark banner center box
            cv2.rectangle(center_overlay, (bx, by), (bx + banner_w, by + banner_h), (20, 10, 10), -1)
            # Glowing neon cyan outline border
            cv2.rectangle(center_overlay, (bx, by), (bx + banner_w, by + banner_h), (255, 255, 0), 2, cv2.LINE_AA)
            
            cv2.addWeighted(center_overlay, 0.8, frame, 0.2, 0, frame)
            
            txt = f"BYE! 👋 ({self.announcement_hand.upper()} HAND)"
                
            # Render Announcement
            t_size, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 0.85, 2)
            tx = bx + (banner_w - t_size[0]) // 2
            ty = by + (banner_h + t_size[1]) // 2
            
            # Drop shadow
            cv2.putText(frame, txt, (tx + 2, ty + 2), cv2.FONT_HERSHEY_DUPLEX, 0.85, (0, 0, 0), 3, cv2.LINE_AA)
            # Bright cyan/yellow text
            cv2.putText(frame, txt, (tx, ty), cv2.FONT_HERSHEY_DUPLEX, 0.85, (255, 255, 0), 2, cv2.LINE_AA)

        # --- D. Top-Center Cooldown Countdown Overlay ---
        cooldown_rem = self.cooldown_until - now
        if cooldown_rem > 0:
            pill_w, pill_h = 220, 32
            px = (w - pill_w) // 2
            py = 95
            
            # semi-transparent dark backing
            overlay = frame.copy()
            cv2.rectangle(overlay, (px, py), (px + pill_w, py + pill_h), (10, 10, 25), -1, cv2.LINE_AA)
            cv2.rectangle(overlay, (px, py), (px + pill_w, py + pill_h), (0, 100, 255), 1, cv2.LINE_AA) # neon orange border
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
            
            # glow text
            cv2.putText(frame, f"⚠️ COOLDOWN: {cooldown_rem:.1f}s", (px + 15, py + 21), 
                        cv2.FONT_HERSHEY_DUPLEX, 0.45, (0, 165, 255), 1, cv2.LINE_AA)

        return frame

    def draw_gauge(self, frame, top_left, width, score):
        """
        Draws a modern horizontal progress bar for wave intensity.
        """
        x, y = top_left
        height = 12
        
        # Draw background bar
        cv2.rectangle(frame, (x, y), (x + width, y + height), (50, 50, 50), -1, cv2.LINE_AA)
        
        # Fill progress bar proportional to score
        fill_width = int(width * score)
        if fill_width > 0:
            # Color gradient: Cyan for full intensity, transition from green
            color = (
                int(255 * score),               # B
                int(255 * (1.0 - score * 0.2)), # G
                int(100 * (1.0 - score))        # R
            )
            cv2.rectangle(frame, (x, y), (x + fill_width, y + height), color, -1, cv2.LINE_AA)
            
        # Draw outline
        cv2.rectangle(frame, (x, y), (x + width, y + height), (150, 150, 150), 1, cv2.LINE_AA)

    def close(self):
        """
        Releases MediaPipe Resources.
        """
        self.hands.close()
        self.pose.close()


def main():
    print("\n=======================================================")
    print("      [VISION ENGINE] MediaPipe Bye Controller Initializing")
    print("=======================================================")
    print("  • Tracks both left and right hands independently.")
    print("  • Tracks body posture & restricts to above-shoulder waving.")
    print("  • Displays rich HSL HUD and neon hand skeletons.")
    print("  • Press 'ESC' or 'Q' in the window to safely exit.")
    print("=======================================================\n")
    
    # Initialize Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot access the webcam (index 0).")
        print("        Please ensure your webcam is connected and not in use.")
        return

    # Set frame dimension constraints
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Instantiate Controller
    controller = ByeGestureController()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARNING] Frame acquisition failed. Retrying...")
                time.sleep(0.03)
                continue
                
            # Process and render
            processed = controller.process_frame(frame)
            
            # Display frame window
            cv2.imshow("AI Bye Controller - MediaPipe Web Vision", processed)
            
            # Non-blocking key check: ESC or 'q' to quit
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
                
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user from console.")
    finally:
        print("\nShutting down vision system...")
        cap.release()
        controller.close()
        cv2.destroyAllWindows()
        print("[SUCCESS] Shutdown completed. Goodbye!")

if __name__ == "__main__":
    main()
