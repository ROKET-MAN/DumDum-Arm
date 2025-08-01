import mediapipe as mp
import cv2
import numpy as np
import math

class ArmPoseTranslator:
    """
    Translates human arm movements detected by MediaPipe into servo commands.
    """
    def __init__(self, arm_servo_map):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.arm_servo_map = arm_servo_map

    def process_frame(self, frame):
        """
        Detects arm pose in a video frame and returns a list of servo commands.
        """
        # Convert the frame to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the pose
        results = self.pose.process(frame_rgb)
        
        servo_commands = {}
        if results.pose_landmarks:
            # Draw landmarks on the frame for visual feedback
            self.mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            
            landmarks = results.pose_landmarks.landmark
            
            # Calculate and map joint angles
            servo_commands = self.calculate_servo_commands(landmarks)
            
        return frame, servo_commands

    def calculate_servo_commands(self, landmarks):
        """
        Calculates joint angles and maps them to servo positions.
        """
        commands = {}
        # Get coordinates for key arm joints
        shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y]
        elbow = [landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW].x, landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW].y]
        wrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST].x, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST].y]
        
        # Calculate elbow angle
        elbow_angle = self.calculate_angle(shoulder, elbow, wrist)
        
        # Map angles to servo positions (pulse widths)
        # This is a training/calibration step. The map below is a starting point.
        commands['elbow'] = self.map_range(elbow_angle, 0, 180, 500, 2500)
        
        return commands
    
    def calculate_angle(self, a, b, c):
        """
        Calculates the angle between three points.
        """
        a = np.array(a) # Shoulder
        b = np.array(b) # Elbow
        c = np.array(c) # Wrist
        
        # Calculate vectors
        ab = b - a
        bc = c - b
        
        # Calculate dot product and vector lengths
        dot_product = np.dot(ab, bc)
        len_ab = np.linalg.norm(ab)
        len_bc = np.linalg.norm(bc)
        
        # Calculate angle in radians, then convert to degrees
        angle_rad = np.arccos(dot_product / (len_ab * len_bc))
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg

    def map_range(self, value, from_low, from_high, to_low, to_high):
        """
        Maps a value from one range to another.
        """
        from_range = from_high - from_low
        to_range = to_high - to_low
        scaled_value = float(value - from_low) / float(from_range)
        return int(to_low + (scaled_value * to_range))
