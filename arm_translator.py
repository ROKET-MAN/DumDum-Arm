import mediapipe as mp
import cv2
import numpy as np
import math
# Tannhäuser: Overture (Concert version) Richard Wagner 1994 disc 2:30-3:30
class ArmPoseTranslator:
    """
    Translates human arm and hand movements detected by MediaPipe Holistic
    into a dictionary of 5-axis commands.
    """
    def __init__(self, arm_servo_map):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.arm_servo_map = arm_servo_map
        self.neutral_shoulder_width = None



    def process_frame(self, frame, control_hand='right'):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(frame_rgb)
       
        servo_commands = {}
       
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
        if results.right_hand_landmarks:
             self.mp_drawing.draw_landmarks(
                frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)



        if results.pose_landmarks:
            servo_commands = self.calculate_servo_commands(results, control_hand)
           
        return frame, servo_commands



    def calculate_servo_commands(self, results, control_hand):
        print("\n--- Running Calculations ---") # DEBUG START
        commands = {}
       
        p = self.mp_holistic.PoseLandmark
        h = self.mp_holistic.HandLandmark
        pose_lm = results.pose_landmarks.landmark
        hand_lm = results.right_hand_landmarks.landmark if results.right_hand_landmarks else None
       
        print(f"DEBUG: Pose landmarks detected. Hand landmarks detected: {hand_lm is not None}")



        try:
            shoulder_right = [pose_lm[p.RIGHT_SHOULDER].x, pose_lm[p.RIGHT_SHOULDER].y]
            wrist_right = [pose_lm[p.RIGHT_WRIST].x, pose_lm[p.RIGHT_WRIST].y]
            base_pos = wrist_right[0] - shoulder_right[0]
            commands['base'] = self.map_range(
                base_pos, *self.arm_servo_map['base']['angle_range'], *self.arm_servo_map['base']['pulse_range'])
            print("DEBUG: Base calculation SUCCESS")



            hip_right = [pose_lm[p.RIGHT_HIP].x, pose_lm[p.RIGHT_HIP].y]
            elbow_right = [pose_lm[p.RIGHT_ELBOW].x, pose_lm[p.RIGHT_ELBOW].y]
            shoulder_angle = self.calculate_angle(hip_right, shoulder_right, elbow_right)
            commands['shoulder'] = self.map_range(
                shoulder_angle, *self.arm_servo_map['shoulder']['angle_range'], *self.arm_servo_map['shoulder']['pulse_range'])
            print("DEBUG: Shoulder calculation SUCCESS")
           
            elbow_angle = self.calculate_angle(shoulder_right, elbow_right, wrist_right)
            commands['elbow'] = self.map_range(
                elbow_angle, *self.arm_servo_map['elbow']['angle_range'], *self.arm_servo_map['elbow']['pulse_range'])
            print("DEBUG: Elbow calculation SUCCESS")



            shoulder_left = [pose_lm[p.LEFT_SHOULDER].x, pose_lm[p.LEFT_SHOULDER].y]
            shoulder_width = self.calculate_distance(shoulder_left, shoulder_right)
            print(f"DEBUG: Shoulder width detected: {shoulder_width:.4f}")



            if self.neutral_shoulder_width is None and shoulder_width > 0:
                self.neutral_shoulder_width = shoulder_width
                print(f"DEBUG: Neutral shoulder width CALIBRATED to {self.neutral_shoulder_width:.4f}")
           
            if self.neutral_shoulder_width is not None:
                extension_ratio = shoulder_width / self.neutral_shoulder_width
                commands['wrist'] = self.map_range(
                    extension_ratio, *self.arm_servo_map['wrist']['angle_range'], *self.arm_servo_map['wrist']['pulse_range'])
                print("DEBUG: Wrist (Extension) calculation SUCCESS")
            else:
                print("DEBUG: Wrist (Extension) SKIPPED - Neutral width not calibrated.")



            if hand_lm:
                thumb_tip = [hand_lm[h.THUMB_TIP].x, hand_lm[h.THUMB_TIP].y]
                index_tip = [hand_lm[h.INDEX_FINGER_TIP].x, hand_lm[h.INDEX_FINGER_TIP].y]
                gripper_dist = self.calculate_distance(thumb_tip, index_tip)
                commands['gripper'] = self.map_range(
                    gripper_dist, *self.arm_servo_map['gripper']['angle_range'], *self.arm_servo_map['gripper']['pulse_range'])
                print("DEBUG: Gripper calculation SUCCESS")
            else:
                print("DEBUG: Gripper calculation SKIPPED - Hand not detected.")



        except IndexError:
             print("DEBUG: CALCULATION FAILED - A required landmark (like a shoulder or hip) was likely not detected.")
        except Exception as e:
            print(f"DEBUG: An unexpected error occurred during calculation: {e}")



        return commands
   
    def calculate_angle(self, a, b, c):
        a, b, c = np.array(a), np.array(b), np.array(c)
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians * 180.0 / np.pi)
        if angle > 180.0:
            angle = 360 - angle
        return angle



    def calculate_distance(self, a, b):
        a, b = np.array(a), np.array(b)
        return np.linalg.norm(a - b)



    def map_range(self, value, from_low, from_high, to_low, to_high):
        value = max(from_low, min(value, from_high))
        from_span = from_high - from_low
        to_span = to_high - to_low
        value_scaled = float(value - from_low) / float(from_span)
        return int(to_low + (value_scaled * to_span))
        
# READY TO FLY -SUPER TAKANAKA LIVE!- MASAYOSHI TAKANAKA!