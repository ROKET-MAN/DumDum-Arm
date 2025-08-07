import cv2
import time
import numpy as np
import threading
import json
import sys
import queue
from flask import Flask, Response, render_template_string
from flask_sock import Sock
#The arm_translator.py file MUST BE IN THE SAME FOLDER AS THIS OTHERWISE YOU NEED TO ADD SOMETHING FOR DIRECTORY. 
# ALL FILES WERE MERGED TO ONE FOR EASE OF USE & SO I CAN COPY AND PASTE THIS TO THE GITHUB!!! 
from arm_translator import ArmPoseTranslator

#CONFIG
ARM_SERVO_MAP = {
    'base':     {'id': 1, 'angle_range': [-0.3, 0.3],   'pulse_range': [500, 2500], 'sim_angle_range': [-180, 180]},
    'shoulder': {'id': 2, 'angle_range': [45, 135],    'pulse_range': [1000, 2000], 'sim_angle_range': [-90, 90]},
    'elbow':    {'id': 3, 'angle_range': [30, 160],    'pulse_range': [700, 2000], 'sim_angle_range': [0, 156]},
    'wrist':    {'id': 4, 'angle_range': [0.8, 1.2],   'pulse_range': [1000, 2000], 'sim_angle_range': [-138, 138]},
    'gripper':  {'id': 5, 'angle_range': [0.02, 0.15], 'pulse_range': [600, 1500], 'sim_angle_range': [0.005, 0.04]}
}

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

#FLASK/SOCK
app = Flask(__name__)
sock = Sock(app)

#SHARED DATA 
shared_data = {
    'latest_frame': None,
    'latest_commands': {},
    'lock': threading.Lock()
}

#LIVE TERMINAL/LOGS
log_queue = queue.Queue()
class LogQueueHandler:
    def write(self, msg):
        log_queue.put(msg)
    def flush(self):
        pass #



# BACKGROUND CAMERA THREAD
def camera_and_ai_thread():
    """Continuously captures frames and runs pose estimation."""
    translator = ArmPoseTranslator(arm_servo_map=ARM_SERVO_MAP)
    video_capture = cv2.VideoCapture(CAMERA_INDEX)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
   
    if not video_capture.isOpened():
        print("CRITICAL: Failed to open camera.")
        return

    while True:
        success, frame = video_capture.read()
        if not success:
            time.sleep(0.01) 
            continue
           
        annotated_frame, servo_commands = translator.process_frame(frame)
       
        with shared_data['lock']:
            shared_data['latest_frame'] = annotated_frame
            if servo_commands:
                shared_data['latest_commands'] = servo_commands
        # processing time will naturally limit the loop speed SO YOUR ORIN DOESNT CRASH!! it happens.

#FLASK ROUTES
@app.route('/')
def index():
    """Serves the main HTML page with the integrated UI."""
    return render_template_string(HTML_PAGE)



def video_stream_generator():
    """Streams the latest annotated frame from the shared data."""
    while True:
        with shared_data['lock']:
            frame = shared_data['latest_frame']
       
        if frame is None:
            time.sleep(0.01)
            continue
       
        #Lowered JPEG quality to 75 to reduces latency, edit to your liking.
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)
       
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
       
    
@app.route('/video_feed')
def video_feed():
    return Response(video_stream_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@sock.route('/ws_commands')
def ws_commands(ws):
    """Handles the WebSocket connection for sending arm commands."""
    print("WebSocket client connected for commands.")
    while True:
        with shared_data['lock']:
            commands = shared_data['latest_commands']
       
        try:
            if commands:
                ws.send(json.dumps(commands))
        except Exception as e:
            print(f"Command WebSocket error: {e}. Client disconnected.")
            break
        # Keeping this sleep to avoid flooding the arm controller with commands
        time.sleep(0.05)
   
@sock.route('/ws_logs')
def ws_logs(ws):
    """Streams captured logs from the queue to the client."""
    print("WebSocket client connected for logs.")
    while True:
        try:
            log_entry = log_queue.get()
            ws.send(log_entry)
        except Exception as e:
            break


#MERGED HTML, CSS, JAVASCRIPT FOR THE WEBPAGE (brought in with gemini as the base then edited to look and be better).
#Vibe code all you want but know when you should mold the statue to your liking.
#AI makes coding faster and leess of a pain. Use it as a vessel to get you far. 
#The car pushed out the need for horses but made transportation faster and more efficient.
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Dum Dum - Live Control</title>
    <style>
        @font-face {
            font-family: 'VCR OSD Mono';
            src: url('https://fonts.cdnfonts.com/s/24847/VCR_OSD_MONO_1.001.woff') format('woff');
        }
        body {
            margin: 0; font-family: 'VCR OSD Mono', monospace; background-color: #111;
            color: white; display: flex; height: 100vh; overflow: hidden;
        }
        .container { display: flex; width: 100%; height: 100%; }
        .left-column { flex: 3; display: flex; flex-direction: column; min-width: 0; z-index: 5; }
        .panel { padding: 10px; box-sizing: border-box; overflow: hidden; }
        .panel h2, .panel h3 { margin: 0 0 10px 0; text-transform: uppercase; letter-spacing: 2px; font-weight: 300; }
        #video_panel { flex: 3; display: flex; flex-direction: column; align-items: center; justify-content: center; }
        #video_panel img { max-width: 100%; max-height: 100%; border: 2px solid #333; object-fit: contain; }
        #log_panel { flex: 1; display: flex; flex-direction: column; border-top: 1px solid #444; background-color: #1a1a1a; align-items: flex-start; }
        #log-output { width: 100%; height: 100%; overflow-y: auto; white-space: pre-wrap; word-wrap: break-word; margin: 0; color: #FFDAB9; font-size: 1em; line-height: 1.2em; }
        #sim_panel_container { flex: 2; border-left: 1px solid #444; position: relative; height: 100%; }
        #sim_canvas { width: 100%; height: 100%; display: block; }
        #sidebar {
            position: absolute; top: 10px; left: 10px; padding: 1rem; background: rgba(0, 0, 0, 0.6);
            backdrop-filter: blur(5px); color: white; border-radius: 8px; border: 1px solid rgba(255, 255, 255, 0.2);
            width: 350px; max-height: calc(100vh - 20px); overflow-y: auto; transition: transform 0.3s ease-in-out; z-index: 10;
        }
        #sidebar.closed { transform: translateX(-110%); }
        #toggle-button {
            position: absolute; top: 10px; left: 10px; background: rgba(0, 0, 0, 0.6); color: white;
            border: 1px solid rgba(255, 255, 255, 0.2); border-radius: 8px; padding: 0.5rem 1rem;
            cursor: pointer; z-index: 11; transition: transform 0.3s ease-in-out; font-family: 'VCR OSD Mono', monospace;
        }
        .control-section { margin-bottom: 1rem; border-top: 1px solid rgba(255, 255, 255, 0.2); padding-top: 1rem; }
        .joint-control { margin-bottom: 0.5rem; }
        label { display: inline-block; margin-bottom: 0.25rem; font-size: 0.9rem; vertical-align: middle; text-transform: capitalize;}
       
        .joint-label {
            color: #fff; background: rgba(0, 0, 0, 0.7); padding: 4px 8px; border-radius: 4px;
            font-size: 14px; white-space: nowrap; text-transform: capitalize; font-family: 'VCR OSD Mono', monospace;
        }



        input[type='range'] {
            -webkit-appearance: none;
            width: 100%;
            background: transparent;
        }
        input[type='range']::-webkit-slider-thumb {
            -webkit-appearance: none;
            height: 16px;
            width: 16px;
            border-radius: 50%;
            background: #FFDAB9;
            cursor: pointer;
            margin-top: -6px;
        }
        input[type='range']::-webkit-slider-runnable-track {
            width: 100%;
            height: 4px;
            cursor: pointer;
            background: #444;
            border-radius: 2px;
        }
        input[type='range']:focus::-webkit-slider-thumb {
            box-shadow: 0 0 0 3px rgba(255, 218, 185, 0.5);
        }
        input[type='range']::-moz-range-thumb {
            height: 16px;
            width: 16px;
            border-radius: 50%;
            background: #FFDAB9;
            cursor: pointer;
            border: none;
        }
        input[type='range']::-moz-range-track {
            width: 100%;
            height: 4px;
            cursor: pointer;
            background: #444;
            border-radius: 2px;
        }
       
        .label-toggle {
            margin-bottom: 0.5rem;
            display: flex;
            align-items: center;
        }
        .label-toggle input[type="checkbox"] {
            display: none;
        }
        .label-toggle label {
            position: relative;
            padding-left: 28px;
            cursor: pointer;
            line-height: 20px;
        }
        .label-toggle label::before {
            content: '';
            position: absolute;
            left: 0;
            top: 0;
            width: 16px;
            height: 16px;
            border: 2px solid #888;
            border-radius: 50%;
            background-color: transparent;
            transition: all 0.2s;
        }
        .label-toggle input[type="checkbox"]:checked + label::before {
            background-color: #FFDAB9;
            border-color: #FFDAB9;
        }



    </style>
</head>
<body>
    <div class="container">
        <div class="left-column">
            <div id="video_panel" class="panel">
                <h2>Live Camera Feed</h2>
                <img src="/video_feed" alt="Live video feed from the robot.">
            </div>
            <div id="log_panel" class="panel">
                <h2>Terminal Output</h2>
                <pre id="log-output"></pre>
            </div>
        </div>
        <div id="sim_panel_container">
            <button id="toggle-button">Toggle Panel</button>
            <div id="sidebar">
                <h2>5-AXIS ARM</h2>
                <p>Arm is controled by DumDum replicating your movement (basically mediapipe but now it turns 2d data to 3d).Use the bars for manual override, DumDum is a SPAZ!!!</p>
                <div class="control-section">
                    <h3>Joint Controls</h3>
                    <div id="sliders-container"></div>
                </div>
                <div class="control-section">
                    <h3>Label Visibility</h3>
                    <div id="toggles-container"></div>
                </div>
            </div>
            <canvas id="sim_canvas"></canvas>
        </div>
    </div>



    <script type="importmap">
    {
        "imports": {
            "three": "https://cdn.jsdelivr.net/npm/three@0.161.0/build/three.module.js",
            "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.161.0/examples/jsm/"
        }
    }
    </script>



    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import { RGBELoader } from 'three/addons/loaders/RGBELoader.js';
        import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';



        const ARM_SERVO_MAP = JSON.parse(`{{ arm_map_json | tojson | safe }}`);
        const UI_JOINT_ORDER = ['base', 'shoulder', 'elbow', 'wrist', 'gripper'];
       
        const targetJointsState = UI_JOINT_ORDER.map(key => {
            const sim_range = ARM_SERVO_MAP[key].sim_angle_range;
            return {
                name: key,
                angle: (sim_range[0] + sim_range[1]) / 2,
                min: sim_range[0],
                max: sim_range[1]
            };
        });
       
        const currentJointsState = JSON.parse(JSON.stringify(targetJointsState));



        const jointLabels = {};
        const robotParts = {};
        let isAiControlActive = true;
        let lastUserInteractionTime = 0;



        const simContainer = document.getElementById('sim_panel_container');
        const canvas = document.getElementById('sim_canvas');
        const scene = new THREE.Scene();
        const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1;



        const labelRenderer = new CSS2DRenderer();
        labelRenderer.domElement.style.position = 'absolute';
        labelRenderer.domElement.style.top = '0px';
        labelRenderer.domElement.style.pointerEvents = 'none';
        simContainer.appendChild(labelRenderer.domElement);



        const camera = new THREE.PerspectiveCamera(50, simContainer.clientWidth / simContainer.clientHeight, 0.1, 1000);
        camera.position.set(5, 3, 5);
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 1, 0);
        controls.enableDamping = true;



        scene.add(new THREE.AmbientLight(0xffffff, 0.5));
        const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
        dirLight.position.set(10, 10, 5);
        scene.add(dirLight);



        new RGBELoader()
            .setPath('https://cdn.jsdelivr.net/npm/three@0.158.0/examples/textures/equirectangular/')
            .load('venice_sunset_1k.hdr', function (texture) {
                texture.mapping = THREE.EquirectangularReflectionMapping;
                scene.background = texture;
                scene.environment = texture;
            });



        scene.add(new THREE.GridHelper(20, 20, 0x444444, 0x888888));



        function createLimb(geometry, color, position, parent) {
            const material = new THREE.MeshStandardMaterial({ color, metalness: 0.7, roughness: 0.5 });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.position.fromArray(position);
            parent.add(mesh);
            return mesh;
        }



        function createLabel(text, parent, yOffset = 0.2) {
            const div = document.createElement('div');
            div.className = 'joint-label';
            div.textContent = text;
            const label = new CSS2DObject(div);
            label.position.set(0, yOffset, 0);
            parent.add(label);
            jointLabels[text.toLowerCase()] = label;
            return label;
        }



        const robot = new THREE.Group();
        scene.add(robot);
        robotParts.base = new THREE.Group();
        robot.add(robotParts.base);
        createLimb(new THREE.CylinderGeometry(0.3, 0.3, 0.2, 32), '#6B7280', [0, 0.1, 0], robotParts.base);
        createLabel('Base', robotParts.base, 0.3);
        robotParts.shoulder = new THREE.Group();
        robotParts.shoulder.position.set(0, 0.3, 0);
        robotParts.base.add(robotParts.shoulder);
        createLimb(new THREE.BoxGeometry(0.2, 0.8, 0.2), '#9CA3AF', [0, 0.4, 0], robotParts.shoulder);
        createLimb(new THREE.BoxGeometry(0.15, 1.2, 0.15), '#D1D5DB', [0, 0.8, 0], robotParts.shoulder);
        createLabel('Shoulder', robotParts.shoulder, 0.5);
        robotParts.elbow = new THREE.Group();
        robotParts.elbow.position.set(0, 1.4, 0);
        robotParts.shoulder.add(robotParts.elbow);
        createLimb(new THREE.SphereGeometry(0.12, 16, 16), '#9CA3AF', [0, 0.1, 0], robotParts.elbow);
        createLimb(new THREE.BoxGeometry(0.12, 1.2, 0.12), '#D1D5DB', [0, 0.6, 0], robotParts.elbow);
        createLabel('Elbow', robotParts.elbow, 0.8);
        robotParts.wrist = new THREE.Group();
        robotParts.wrist.position.set(0, 1.2, 0);
        robotParts.elbow.add(robotParts.wrist);
        createLimb(new THREE.SphereGeometry(0.1, 16, 16), '#9CA3AF', [0, 0.1, 0], robotParts.wrist);
        createLabel('Wrist', robotParts.wrist, 0.1);
        const gripperGroup = new THREE.Group();
        gripperGroup.position.set(0, 0.2, 0);
        gripperGroup.rotation.y = THREE.MathUtils.degToRad(90);
        robotParts.wrist.add(gripperGroup);
        createLabel('Gripper', gripperGroup, 0.05);
        createLimb(new THREE.BoxGeometry(0.08, 0.08, 0.08), '#6B7280', [0, 0, 0], gripperGroup);
        const fingerGeometry = new THREE.BoxGeometry(0.02, 0.15, 0.05);
        robotParts.finger1 = createLimb(fingerGeometry, '#D1D5DB', [0, 0.075, 0], gripperGroup);
        robotParts.finger2 = createLimb(fingerGeometry, '#D1D5DB', [0, 0.075, 0], gripperGroup);



        const slidersContainer = document.getElementById('sliders-container');
        targetJointsState.forEach(joint => {
            const controlDiv = document.createElement('div');
            controlDiv.className = 'joint-control';
            const sliderLabel = document.createElement('label');
            sliderLabel.id = `label-${joint.name}`;
            const slider = document.createElement('input');
            slider.type = 'range'; slider.min = joint.min; slider.max = joint.max; slider.value = joint.angle; slider.id = `slider-${joint.name}`;
            if (joint.name === 'gripper') {
                slider.step = 0.001;
            } else {
                slider.step = 1;
            }
            slider.oninput = () => {
                isAiControlActive = false;
                lastUserInteractionTime = Date.now();
                joint.angle = parseFloat(slider.value);
            };
            controlDiv.appendChild(sliderLabel); controlDiv.appendChild(slider); slidersContainer.appendChild(controlDiv);
        });
       
        const togglesContainer = document.getElementById('toggles-container');
        Object.keys(jointLabels).forEach(key => {
            const toggleDiv = document.createElement('div');
            toggleDiv.className = 'label-toggle';
            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox'; checkbox.checked = true; checkbox.id = `toggle-${key}`;
            checkbox.onchange = () => { jointLabels[key].visible = checkbox.checked; };
            const checkLabel = document.createElement('label');
            checkLabel.textContent = key; checkLabel.htmlFor = `toggle-${key}`;
            toggleDiv.appendChild(checkbox); toggleDiv.appendChild(checkLabel);
            togglesContainer.appendChild(toggleDiv);
        });
       
        const sidebar = document.getElementById('sidebar');
        const toggleButton = document.getElementById('toggle-button');
        toggleButton.onclick = () => {
            sidebar.classList.toggle('closed');
            const isClosed = sidebar.classList.contains('closed');
            toggleButton.style.transform = isClosed ? 'translateX(0px)' : `translateX(${sidebar.offsetWidth + 10}px)`;
        };



        const cmd_ws = new WebSocket(`ws://${window.location.host}/ws_commands`);
        cmd_ws.onopen = () => console.log("Command WebSocket connected.");
        cmd_ws.onerror = (err) => console.error("Command WebSocket error:", err);
        cmd_ws.onmessage = (event) => {
            if (!isAiControlActive) return;
            const commands = JSON.parse(event.data);
            for (const [jointName, pulse] of Object.entries(commands)) {
                if (ARM_SERVO_MAP[jointName]) {
                    const joint_data = ARM_SERVO_MAP[jointName];
                    const pulse_min = joint_data.pulse_range[0];
                    const pulse_max = joint_data.pulse_range[1];
                    const sim_angle_min = joint_data.sim_angle_range[0];
                    const sim_angle_max = joint_data.sim_angle_range[1];
                    const value_scaled = (pulse - pulse_min) / (pulse_max - pulse_min);
                    const target_value = sim_angle_min + (value_scaled * (sim_angle_max - sim_angle_min));
                    const jointState = targetJointsState.find(j => j.name === jointName);
                    if (jointState) jointState.angle = target_value;
                }
            }
        };
       
        const logOutput = document.getElementById('log-output');
        const log_ws = new WebSocket(`ws://${window.location.host}/ws_logs`);
        log_ws.onopen = () => { logOutput.textContent += '--> Log stream connected.\\n'; };
        log_ws.onerror = (err) => { logOutput.textContent += '--> Log stream connection error!\\n'; };
        log_ws.onmessage = (event) => {
            logOutput.textContent += event.data;
            logOutput.scrollTop = logOutput.scrollHeight;
        };



        function animate() {
            requestAnimationFrame(animate);
            if (!isAiControlActive && (Date.now() - lastUserInteractionTime > 3000)) {
                isAiControlActive = true;
            }
           
            const lerpFactor = 0.1;



            currentJointsState.forEach((currentJoint, index) => {
                const targetJoint = targetJointsState[index];
                currentJoint.angle += (targetJoint.angle - currentJoint.angle) * lerpFactor;



                const part = robotParts[currentJoint.name];
                if (part) {
                    if (currentJoint.name === 'shoulder' || currentJoint.name === 'elbow' || currentJoint.name === 'wrist') {
                        part.rotation.z = THREE.MathUtils.degToRad(currentJoint.angle);
                    } else if (currentJoint.name === 'base') {
                        part.rotation.y = THREE.MathUtils.degToRad(currentJoint.angle);
                    }
                }
               
                const slider = document.getElementById(`slider-${currentJoint.name}`);
                const label = document.getElementById(`label-${currentJoint.name}`);
                if (slider && label) {
                    if (isAiControlActive) {
                        slider.value = currentJoint.angle;
                    }
                    if (currentJoint.name === 'gripper') {
                        label.textContent = `${currentJoint.name}: ${currentJoint.angle.toFixed(3)}`;
                    } else {
                        label.textContent = `${currentJoint.name.replace(/_/g, ' ')}: ${currentJoint.angle.toFixed(1)}°`;
                    }
                }
            });



            const gripperState = currentJointsState.find(j => j.name === 'gripper');
            if (gripperState) {
                const finger_dist = (gripperState.angle - gripperState.min) / (gripperState.max - gripperState.min) * 0.06 + 0.02;
                robotParts.finger1.position.x = -finger_dist;
                robotParts.finger2.position.x = finger_dist;
            }




            const width = simContainer.clientWidth;
            const height = simContainer.clientHeight;
            if (canvas.width !== width || canvas.height !== height) {
                renderer.setSize(width, height, false);
                labelRenderer.setSize(width, height);
                camera.aspect = width / height;
                camera.updateProjectionMatrix();
            }



            controls.update();
            renderer.render(scene, camera);
            labelRenderer.render(scene, camera);
        }
        animate();
    </script>
</body>
</html>
"""

if __name__ == '__main__':
    @app.context_processor
    def inject_globals():
        return {'arm_map_json': ARM_SERVO_MAP}
    sys.stdout = LogQueueHandler()
    print("Starting background thread for camera and AI...")
    main_thread = threading.Thread(target=camera_and_ai_thread)
    main_thread.daemon = True
    main_thread.start()
    print(f"Starting Flask server. Open your browser to http://127.0.0.1:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)

    #600!!!!!!! :D