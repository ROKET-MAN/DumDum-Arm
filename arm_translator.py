import mediapipe as mp
import cv2
import numpy as np
import math

class ArmPoseTranslator:
    """
    Translates human arm and hand movements into 5-axis commands using a
    real-time 3D Inverse Kinematics (IK) solver for Virtual Puppet (VP) control.
    
    VERSION 3.2 UPDATE:
    - Implemented a stable 3D angle calculation for the wrist using vectors
      from the Pose model's world landmarks to fix instability issues.
    - Tuned model complexity and smoothing for lower latency and better performance.
    """
    def __init__(self, arm_servo_map, smoothing_factor=0.75):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands

        # Initialize two separate models with performance optimizations
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=0,  # Use Lite model for speed
            enable_segmentation=False,
            smooth_landmarks=False # Disable internal smoothing, we have our own
        )
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            max_num_hands=1 # We only need to track one hand
        )

        self.arm_servo_map = arm_servo_map
        self.smoothing_factor = smoothing_factor
        self.smoothed_commands = {}
        self.tuning_guide_printed = False

    def process_frame(self, frame, control_hand='right'):
        """
        Processes a single video frame to detect pose and hands,
        then generates servo commands.
        """
        frame.flags.writeable = False
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with BOTH models
        pose_results = self.pose.process(frame_rgb)
        hand_results = self.hands.process(frame_rgb)

        frame.flags.writeable = True
        servo_commands = {}

        # Draw landmarks for visualization from both models
        if pose_results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        
        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # Calculate commands using the results from both models
        if pose_results.pose_world_landmarks:
            servo_commands = self.calculate_servo_commands(pose_results, hand_results, control_hand)

        return frame, servo_commands

    def calculate_servo_commands(self, pose_results, hand_results, control_hand):
        """
        Calculates servo pulses using real-time IK on 3D world coordinates
        from the Pose model and hand landmarks from the Hands model.
        """
        if not self.tuning_guide_printed:
            print("\n--- DUAL MODEL: 3D POSE + HANDS IK SOLVER ACTIVE ---")
            print("Control logic uses BlazePose for arm and Hand model for gripper/wrist.")
            print("IMPORTANT: You may need to re-tune your 'angle_range' values in ARM_SERVO_MAP.")
            print("Watch the 'RAW VALUES' in the terminal to find the new min/max for each joint.")
            print("----------------------------------------------------------------\n")
            self.tuning_guide_printed = True

        commands = {}
        raw_values = {}

        p = self.mp_pose.PoseLandmark
        h = self.mp_hands.HandLandmark

        # Get landmarks from their respective result objects
        pose_lm_world = pose_results.pose_world_landmarks.landmark
        
        # Get the first detected hand's landmarks
        hand_lm = None
        if hand_results.multi_hand_landmarks:
            hand_lm = hand_results.multi_hand_landmarks[0].landmark

        try:
            # KEY 3D WORLD LANDMARKS from the POSE model
            shoulder_r_3d = np.array([pose_lm_world[p.RIGHT_SHOULDER].x, pose_lm_world[p.RIGHT_SHOULDER].y, pose_lm_world[p.RIGHT_SHOULDER].z])
            elbow_r_3d = np.array([pose_lm_world[p.RIGHT_ELBOW].x, pose_lm_world[p.RIGHT_ELBOW].y, pose_lm_world[p.RIGHT_ELBOW].z])
            wrist_r_3d = np.array([pose_lm_world[p.RIGHT_WRIST].x, pose_lm_world[p.RIGHT_WRIST].y, pose_lm_world[p.RIGHT_WRIST].z])

            # 1. BASE (YAW)
            base_pos = wrist_r_3d[0]
            raw_values['base'] = base_pos
            commands['base'] = self.map_range(base_pos, *self.arm_servo_map['base']['angle_range'], *self.arm_servo_map['base']['pulse_range'])

            # 2. & 3. SHOULDER AND ELBOW (Real-time IK)
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
            horizontal_dist = np.linalg.norm([target_vector[0], target_vector[2]])
            base_pitch_rad = np.arctan2(-target_vector[1], horizontal_dist)
            
            if upper_arm_dist > 0 and target_dist > 0:
                cos_shoulder_angle = (upper_arm_dist**2 + target_dist**2 - lower_arm_dist**2) / (2 * upper_arm_dist * target_dist)
                cos_shoulder_angle = np.clip(cos_shoulder_angle, -1.0, 1.0)
                ik_shoulder_angle_rad = np.arccos(cos_shoulder_angle)
                final_shoulder_rad = base_pitch_rad + ik_shoulder_angle_rad
                shoulder_angle_deg = np.degrees(final_shoulder_rad)
                raw_values['shoulder'] = shoulder_angle_deg
                commands['shoulder'] = self.map_range(shoulder_angle_deg, *self.arm_servo_map['shoulder']['angle_range'], *self.arm_servo_map['shoulder']['pulse_range'])

            # 4. & 5. WRIST and GRIPPER
            
            # WRIST (Pitch) - STABLE 3D ANGLE METHOD
            # We calculate a true 3D angle using only landmarks from the Pose model,
            # which are all in the same real-world coordinate space.
            elbow_w = np.array([pose_lm_world[p.RIGHT_ELBOW].x, pose_lm_world[p.RIGHT_ELBOW].y, pose_lm_world[p.RIGHT_ELBOW].z])
            wrist_w = np.array([pose_lm_world[p.RIGHT_WRIST].x, pose_lm_world[p.RIGHT_WRIST].y, pose_lm_world[p.RIGHT_WRIST].z])
            index_w = np.array([pose_lm_world[p.RIGHT_INDEX].x, pose_lm_world[p.RIGHT_INDEX].y, pose_lm_world[p.RIGHT_INDEX].z])

            # Create vectors for the forearm and hand
            v_forearm = wrist_w - elbow_w
            v_hand = index_w - wrist_w
            
            # Normalize the vectors to get direction only
            v_forearm_norm = v_forearm / np.linalg.norm(v_forearm)
            v_hand_norm = v_hand / np.linalg.norm(v_hand)
            
            # Calculate the angle between them using the dot product
            dot_product = np.dot(v_forearm_norm, v_hand_norm)
            dot_product = np.clip(dot_product, -1.0, 1.0) # Clip for safety
            angle_rad = np.arccos(dot_product)
            wrist_angle_deg = np.degrees(angle_rad)
            
            raw_values['wrist'] = wrist_angle_deg
            commands['wrist'] = self.map_range(wrist_angle_deg, *self.arm_servo_map['wrist']['angle_range'], *self.arm_servo_map['wrist']['pulse_range'])

            # GRIPPER - Uses the Hand model exclusively
            if hand_lm:
                thumb_tip = np.array([hand_lm[h.THUMB_TIP].x, hand_lm[h.THUMB_TIP].y, hand_lm[h.THUMB_TIP].z])
                index_tip = np.array([hand_lm[h.INDEX_FINGER_TIP].x, hand_lm[h.INDEX_FINGER_TIP].y, hand_lm[h.INDEX_FINGER_TIP].z])
                middle_tip = np.array([hand_lm[h.MIDDLE_FINGER_TIP].x, hand_lm[h.MIDDLE_FINGER_TIP].y, hand_lm[h.MIDDLE_FINGER_TIP].z])
                pinch_center = (index_tip + middle_tip) / 2
                gripper_dist = np.linalg.norm(thumb_tip - pinch_center)
                raw_values['gripper'] = gripper_dist
                commands['gripper'] = self.map_range(gripper_dist, *self.arm_servo_map['gripper']['angle_range'], *self.arm_servo_map['gripper']['pulse_range'])

            # Smoothing Filter
            for joint, value in commands.items():
                last_smoothed = self.smoothed_commands.get(joint, value)
                smoothed_value = int(self.smoothing_factor * value + (1 - self.smoothing_factor) * last_smoothed)
                self.smoothed_commands[joint] = smoothed_value

            print(
                f"RAW VALUES (3D Pose+Hand): Base(m): {raw_values.get('base', 0):.2f} | "
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
        value = max(from_low, min(value, from_high))
        from_span = from_high - from_low
        to_span = to_high - to_low
        if from_span == 0: return to_low
        value_scaled = float(value - from_low) / float(from_span)
        mapped_value = to_low + (value_scaled * to_span)
        return int(max(min(to_low, to_high), min(max(to_low, to_high), mapped_value)))