import sys
import os

# --- IN-CODE PATH WORKAROUND ---
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.append(project_root)
# --- END WORKAROUND ---

from bleak import BleakClient
import asyncio
import cv2
import numpy as np
import mediapipe as mp
from arm_translator import ArmPoseTranslator
from ble_controller import format_command

from flask import Flask, Response
from flask_cors import CORS

# --- CONFIGURATION FLAG ---
BLUETOOTH_ENABLED = False
MINIARM_ADDRESS = "48:87:2D:68:1C:4F"
WRITE_CHAR_HANDLE = 32

# --- CAMERA CONFIGURATION ---
# Change this number to match the index of your camera from the 'ls /dev/video*' command.
CAMERA_DEVICE_INDEX = 1 

# --- FLASK WEB SERVER SETUP ---
app = Flask(__name__)
CORS(app)

# Global variables for camera feed and pose translator
cap = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
translator = ArmPoseTranslator({})

def generate_frames():
    """Generator function to stream camera frames."""
    if not cap.isOpened():
        print("Error: Could not open video stream in generate_frames.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Process the frame with the AI
        frame_with_landmarks, _ = translator.process_frame(frame)

        # Encode the frame as a JPEG image
        ret, buffer = cv2.imencode('.jpg', frame_with_landmarks)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()

        # Yield the frame in a streaming response
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Feeds the camera frames."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "H.U.E. AI Arm Translator is running. Go to /video_feed to see the camera."

def run_flask_server():
    """Function to run the Flask server."""
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Warning: This script must be run with sudo for BLE communication.")
        print("Please run with: sudo python3 /home/nvidia/hue_ws/src/dumdum.py")
        sys.exit(1)
    
    print("Bluetooth connection skipped. Running in camera-only mode.")
    print("Starting Flask web server...")
    run_flask_server()
