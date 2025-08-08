# DumDum - 5-Axis Robot Arm Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

DumDum is a real-time, vision-based controller for a 5-axis robot arm. It uses a combination of MediaPipe Pose (BlazePose) and MediaPipe Hands to translate your arm and hand movements into (near) precise robotic commands. Then it's all visualized through a live, interactive web interface.



---

##  Features

* **Real-Time 3D Pose Tracking:** Uses Google's MediaPipe Pose and Hands to track your arm's position in 3D space with mild accuracy (needs better depth).
* **5-Axis Control:** Independently controls Base (Yaw), Shoulder, Elbow, Wrist (Pitch), and a Gripper.
* **Advanced IK Solver:** Employs a real-time Inverse Kinematics (IK) solver to calculate complex shoulder and elbow angles.
* **Intelligent Smoothing:** A One-Euro Filter is used to eliminate camera jitter, providing exceptionally smooth and responsive motion.
* **Interactive Web UI:** A browser-based dashboard built with Flask and Three.js shows a live video feed, terminal logs, and a 3D simulation of the robot arm that mirrors your movements. (3D model based on [temirlanzzz's Robot_pose_estimation arm](https://github.com/temirlanzzz/robot_pose_estimation))
* **Highly Tunable:** Configuration for servo ranges, sensitivity, and performance is easily accessible and can be fine-tuned to your liking (needs work).

---

##  How It Works

The system is a "virtual puppeteer" that follows a simple three-step process: **See, Think, and Act.**

1.  **SEE (Input & Vision):**
    * A webcam watches your movements.
    * Two MediaPipe AI models work together: one for your body skeleton and one for your hand, creating a live 3D digital skeleton by converting the webcam's 2D data to 3D coordinates.

2.  **THINK (Calculation & Filtering):**
    * The script performs real-time calculations on the 3D skeleton to determine the precise angles for each arm joint.
    * These calculated values are passed through the One-Euro Filter to smooth out shakiness without adding lag.

3.  **ACT (Output & Visualization):**
    * The final, smooth commands are mapped to servo pulse values.
    * These commands are sent to the web interface via WebSocket to animate the 3D model.
    * The live video feed with the skeleton drawn on top is streamed to the browser.

---

##  Getting Started

Follow these instructions to get the project running on your local machine.

### Prerequisites

* Python 3.8 or newer
* Git command-line tools
* A webcam
* Orin Nano (or a similar capable device)

### Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/your-username/your-repository-name.git](https://github.com/your-username/your-repository-name.git)
    cd your-repository-name
    ```

2.  **Create a Python virtual environment (recommended):**
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use: venv\Scripts\activate
    ```

3.  **Create a `requirements.txt` file:**
    Create a new file named `requirements.txt` in your project folder and add the following lines to it:
    ```
    numpy<2.0
    opencv-python
    mediapipe
    flask
    flask-sock
    ```

4.  **Install the dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

### Running the Application

1.  **Start the web server:**
    ```bash
    python dumdum_web_interface.py
    ```

2.  **Open the interface:**
    Open your web browser and navigate to `http://127.0.0.1:5000`. You should see the live video feed and 3D simulation.

---

## ⚙onfiguration & Tuning

The most important configuration is in the `dumdum_web_interface.py` file.

### `ARM_SERVO_MAP`

This Python dictionary is crucial for calibrating the system to your specific robot arm.

ARM_SERVO_MAP = {
    'base':     {'id': 1, 'angle_range': [-0.3, 0.3], 'pulse_range': [500, 2500]},
    'shoulder': {'id': 2, 'angle_range': [45, 135],  'pulse_range': [1000, 2000]},
    # ... and so on for each joint
}

angle_range: This is the most important part to tune. It defines the range of the raw input values (meters for base, degrees for joints, distance for gripper) from your body's movement. Watch the RAW VALUES printed in the terminal to find the min and max values for your personal range of motion, and update this range for each joint.

pulse_range: This is the minimum and maximum pulse width (in microseconds) that your virtual servo motor accepts (adapt to fit your physical motor).

## Performance Tuning
For more advanced tuning, you can modify parameters inside arm_translator.py:

model_complexity: In the Pose model initialization, can be 0 (lite), 1 (full), or 2 (heavy) for a trade-off between speed and accuracy.

OneEuroFilter(min_cutoff, beta): These parameters in the __init__ method control the smoothing. Lowering min_cutoff will increase smoothing when still, while increasing beta will make the arm more responsive to fast movements.

## File Structure
dumdum/
├── arm_translator.py       # The "Brain": All AI, IK, and filtering logic.
├── dumdum_web_interface.py # The "Face": Flask server, UI, and main execution script.
└── README.md               # This file.
## Acknowledgements
This project stands on the shoulders of giants. A huge thank you to the teams behind the models and resources. Personal thank you to Brighton Gannaway:

Google MediaPipe

Flask

Three.js

temirlanzzz/robot_pose_estimation
