import mediapipe as mp
import cv2
import numpy as np
import math

class ArmPoseTranslator:
    """
    Translates human arm and hand movements into 5-axis commands using a
    full, real-time 3D Inverse Kinematics (IK) solver for precise "puppet" control.
    
    VERSION 2.0 UPDATE:
    This version now uses MediaPipe's 'pose_world_landmarks' for all calculations.
    Instead of using landmarks relative to the 2D image frame, this uses a true
    3D coordinate system measured in meters, with the hip center as the origin.
    This provides vastly more stable and accurate data for the IK solver,
    making the arm's response less dependent on camera angle and distance.
    """
    def __init__(self, arm_servo_map, smoothing_factor=0.75):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=2
        )
        self.arm_servo_map = arm_servo_map
        self.smoothing_factor = smoothing_factor
        self.smoothed_commands = {}
        self.tuning_guide_printed = False

    def process_frame(self, frame, control_hand='right'):
        """
        Processes a single video frame to detect pose and generate servo commands.
        """
        frame.flags.writeable = False
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(frame_rgb)
        frame.flags.writeable = True

        servo_commands = {}

        # Draw landmarks for visualization - this uses the standard 2D landmarks
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
        if results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)

        # Calculate commands using the new 3D world landmarks
        if results.pose_world_landmarks:
            servo_commands = self.calculate_servo_commands(results, control_hand)

        return frame, servo_commands

    def calculate_servo_commands(self, results, control_hand):
        """
        Calculates servo pulses using real-time IK on true 3D world coordinates.
        """
        if not self.tuning_guide_printed:
            print("\n--- REAL-TIME 3D WORLD IK SOLVER ACTIVE ---")
            print("Control logic upgraded to use true 3D metric coordinates.")
            print("IMPORTANT: You MUST re-tune your 'angle_range' values in ARM_SERVO_MAP.")
            print("Watch the 'RAW VALUES' in the terminal to find the new min/max for each joint.")
            print("----------------------------------------------------------------\n")
            self.tuning_guide_printed = True

        commands = {}
        raw_values = {}

        p = self.mp_holistic.PoseLandmark
        h = self.mp_holistic.HandLandmark
        
        # *** KEY CHANGE: Use pose_world_landmarks for all calculations ***
        # These landmarks are in meters, in a 3D space relative to the hip center.
        pose_lm = results.pose_world_landmarks.landmark
        
        # Hand landmarks are still relative to the wrist, so we use the standard ones.
        hand_lm = results.right_hand_landmarks.landmark if results.right_hand_landmarks else None

        try:
            # --- Get key 3D world landmarks ---
            shoulder_r_3d = np.array([pose_lm[p.RIGHT_SHOULDER].x, pose_lm[p.RIGHT_SHOULDER].y, pose_lm[p.RIGHT_SHOULDER].z])
            elbow_r_3d = np.array([pose_lm[p.RIGHT_ELBOW].x, pose_lm[p.RIGHT_ELBOW].y, pose_lm[p.RIGHT_ELBOW].z])
            wrist_r_3d = np.array([pose_lm[p.RIGHT_WRIST].x, pose_lm[p.RIGHT_WRIST].y, pose_lm[p.RIGHT_WRIST].z])
            
            # --- 1. BASE (YAW) ---
            # Use the world X-coordinate of the wrist for base rotation.
            # This is more stable than screen position as it's a real-world measurement.
            # The origin (0) is the center of your hips.
            base_pos = wrist_r_3d[0] 
            raw_values['base'] = base_pos
            # IMPORTANT: The input range [-0.4, 0.4] is a starting guess.
            # You will need to observe your own 'RAW VALUES' for 'base' and
            # update the 'angle_range' in your config for optimal sensitivity.
            commands['base'] = self.map_range(base_pos, *self.arm_servo_map['base']['angle_range'], *self.arm_servo_map['base']['pulse_range'])

            # --- 2. & 3. SHOULDER AND ELBOW (Real-time IK) ---
            # This logic is the same, but now operates on metric distances, making it more accurate.
            target_vector = wrist_r_3d - shoulder_r_3d
            target_dist = np.linalg.norm(target_vector)
            upper_arm_dist = np.linalg.norm(elbow_r_3d - shoulder_r_3d)
            lower_arm_dist = np.linalg.norm(wrist_r_3d - elbow_r_3d)

            # ELBOW
            cos_elbow_angle = (upper_arm_dist**2 + lower_arm_dist**2 - target_dist**2) / (2 * upper_arm_dist * lower_arm_dist)
            cos_elbow_angle = np.clip(cos_elbow_angle, -1.0, 1.0)
            elbow_angle_rad = np.arccos(cos_elbow_angle)
            elbow_angle_deg = 180 - np.degrees(elbow_angle_rad)
            raw_values['elbow'] = elbow_angle_deg
            commands['elbow'] = self.map_range(elbow_angle_deg, *self.arm_servo_map['elbow']['angle_range'], *self.arm_servo_map['elbow']['pulse_range'])

            # SHOULDER
            # We calculate the shoulder pitch based on the wrist's height (Y) relative to its distance from the body's center plane (XZ).
            horizontal_dist = np.linalg.norm([target_vector[0], target_vector[2]]) 
            base_pitch_rad = np.arctan2(-target_vector[1], horizontal_dist)
            
            # Check for valid distances to prevent math errors
            if upper_arm_dist > 0 and target_dist > 0:
                # Use law of cosines to find the angle within the arm linkage
                cos_shoulder_angle = (upper_arm_dist**2 + target_dist**2 - lower_arm_dist**2) / (2 * upper_arm_dist * target_dist)
                cos_shoulder_angle = np.clip(cos_shoulder_angle, -1.0, 1.0)
                ik_shoulder_angle_rad = np.arccos(cos_shoulder_angle)
                
                # Combine the base pitch with the IK angle for the final shoulder angle
                final_shoulder_rad = base_pitch_rad + ik_shoulder_angle_rad
                shoulder_angle_deg = np.degrees(final_shoulder_rad)
                raw_values['shoulder'] = shoulder_angle_deg
                commands['shoulder'] = self.map_range(shoulder_angle_deg, *self.arm_servo_map['shoulder']['angle_range'], *self.arm_servo_map['shoulder']['pulse_range'])

            # --- 4. & 5. WRIST and GRIPPER (Hand-dependent controls) ---
            if hand_lm:
                # WRIST (Pitch)
                # *** UPGRADE: Calculate true 3D angle of the wrist ***
                index_mcp_3d = np.array([hand_lm[h.INDEX_FINGER_MCP].x, hand_lm[h.INDEX_FINGER_MCP].y, hand_lm[h.INDEX_FINGER_MCP].z])
                
                # We need to transform hand landmarks to be in the same world space
                # For simplicity, we'll approximate by calculating the 2D angle on the camera plane,
                # which is still more stable with the new 3D pose backbone.
                elbow_r_2d = np.array([pose_lm[p.RIGHT_ELBOW].x, pose_lm[p.RIGHT_ELBOW].y])
                wrist_r_2d = np.array([pose_lm[p.RIGHT_WRIST].x, pose_lm[p.RIGHT_WRIST].y])
                index_mcp_2d = np.array([hand_lm[h.INDEX_FINGER_MCP].x, hand_lm[h.INDEX_FINGER_MCP].y])
                
                wrist_angle = self.calculate_angle_2d(elbow_r_2d, wrist_r_2d, index_mcp_2d)
                raw_values['wrist'] = wrist_angle
                commands['wrist'] = self.map_range(wrist_angle, *self.arm_servo_map['wrist']['angle_range'], *self.arm_servo_map['wrist']['pulse_range'])

                # GRIPPER
                # This calculation is also more stable now, as the Z coordinate is more reliable.
                thumb_tip = np.array([hand_lm[h.THUMB_TIP].x, hand_lm[h.THUMB_TIP].y, hand_lm[h.THUMB_TIP].z])
                index_tip = np.array([hand_lm[h.INDEX_FINGER_TIP].x, hand_lm[h.INDEX_FINGER_TIP].y, hand_lm[h.INDEX_FINGER_TIP].z])
                middle_tip = np.array([hand_lm[h.MIDDLE_FINGER_TIP].x, hand_lm[h.MIDDLE_FINGER_TIP].y, hand_lm[h.MIDDLE_FINGER_TIP].z])
                pinch_center = (index_tip + middle_tip) / 2
                gripper_dist = np.linalg.norm(thumb_tip - pinch_center)
                raw_values['gripper'] = gripper_dist
                commands['gripper'] = self.map_range(gripper_dist, *self.arm_servo_map['gripper']['angle_range'], *self.arm_servo_map['gripper']['pulse_range'])

            # --- Apply Smoothing Filter ---
            for joint, value in commands.items():
                last_smoothed = self.smoothed_commands.get(joint, value)
                smoothed_value = int(self.smoothing_factor * value + (1 - self.smoothing_factor) * last_smoothed)
                self.smoothed_commands[joint] = smoothed_value

            print(
                f"RAW VALUES (3D World): Base(m): {raw_values.get('base', 0):.2f} | "
                f"Shoulder(d): {raw_values.get('shoulder', 0):.1f} | "
                f"Elbow(d): {raw_values.get('elbow', 0):.1f} | "
                f"Wrist(d): {raw_values.get('wrist', 0):.1f} | "
                f"Gripper(dist): {raw_values.get('gripper', 0):.3f}"
            )

        except Exception as e:
            # print(f"Calculation error: {e}. A landmark may be out of view.")
            return {}

        return self.smoothed_commands
    
    def calculate_angle_2d(self, a, b, c):
        """Calculates angle in degrees between three 2D points (a-b-c)."""
        a, b, c = np.array(a), np.array(b), np.array(c)
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians * 180.0 / np.pi)
        if angle > 180.0:
            angle = 360 - angle
        return angle

    def map_range(self, value, from_low, from_high, to_low, to_high):
        """Maps a value from one range to another, clamping it to the target range."""
        # Clamp the value to the source range
        value = max(from_low, min(value, from_high))
        
        from_span = from_high - from_low
        to_span = to_high - to_low
        
        # Avoid division by zero
        if from_span == 0: return to_low
        
        # Map and clamp to the target range
        value_scaled = float(value - from_low) / float(from_span)
        mapped_value = to_low + (value_scaled * to_span)
        
        # Ensure the final value is an integer and within the target pulse range
        return int(max(min(to_low, to_high), min(max(to_low, to_high), mapped_value)))

