#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Integrated Motor Control + LiDAR + Camera Server
Features:
- Individual motor control (Motor1: F/B, Motor2: L/R)
- Real-time LiDAR mapping (D200 via serial)
- Pi Camera streaming
- Web interface with virtual joystick
"""

from flask import Flask, render_template, request, Response
from flask_socketio import SocketIO, emit
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time
import threading
import socket
import json
import serial
import struct
import collections
import math
import base64
import io

# Camera imports
try:
    from picamera2 import Picamera2
    import cv2
    CAMERA_AVAILABLE = True
    print("Camera imports successful")
except ImportError as e:
    print(f"Camera import failed: {e}")
    print("Run: sudo apt install python3-picamera2 python3-opencv")
    CAMERA_AVAILABLE = False

# ===== MOTOR CONFIGURATION =====
DIR1_PIN = 18   # Motor 1 Direction (Forward/Backward)
PWM1_PIN = 19   # Motor 1 Speed (Forward/Backward)
DIR2_PIN = 20   # Motor 2 Direction (Left/Right)
PWM2_PIN = 21   # Motor 2 Speed (Left/Right)

# ===== LIDAR CONFIGURATION =====
LIDAR_PORT = "/dev/ttyUSB0"  # Change to your LiDAR port
LIDAR_BAUDRATE = 230400
FRAME_LEN = 47
HEADER = 0x54
POINTS_PER_PACK = 12
LIDAR_SIMULATE = True  # Set False when hardware connected

# LiDAR CRC Table (from D200 documentation)
CRC_TABLE = [
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
]

print("Integrated Motor + LiDAR + Camera Control Server")
print(f"Motor 1 (F/B): DIR={DIR1_PIN}, PWM={PWM1_PIN}")
print(f"Motor 2 (L/R): DIR={DIR2_PIN}, PWM={PWM2_PIN}")
print(f"LiDAR: {LIDAR_PORT} @ {LIDAR_BAUDRATE}")

# ===== Initialize Motors =====
motors_ready = False
try:
    motor1_dir = DigitalOutputDevice(DIR1_PIN)
    motor2_dir = DigitalOutputDevice(DIR2_PIN)
    motor1_pwm = PWMOutputDevice(PWM1_PIN, frequency=1000)
    motor2_pwm = PWMOutputDevice(PWM2_PIN, frequency=1000)
    motors_ready = True
    print("? Motors initialized successfully")
except Exception as e:
    print(f"? Motor initialization failed: {e}")
    motors_ready = False

# ===== Initialize Camera =====
camera = None
if CAMERA_AVAILABLE:
    try:
        camera = Picamera2()
        camera_config = camera.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            lores={"size": (320, 240), "format": "YUV420"}
        )
        camera.configure(camera_config)
        camera.start()
        print("? Camera initialized successfully")
        time.sleep(2)  # Let camera warm up
    except Exception as e:
        print(f"? Camera initialization failed: {e}")
        camera = None

# ===== Flask Setup =====
app = Flask(__name__)
app.config['SECRET_KEY'] = 'integrated_control_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ===== Global State =====
current_state = {
    "joystick": {"x": 0, "y": 0, "active": False},
    "motors": {"motor1_speed": 0, "motor1_dir": "stop", "motor2_speed": 0, "motor2_dir": "stop"},
    "gps": {"lat": 0, "lon": 0, "accuracy": 0, "available": False},
    "lidar": {"points": [], "last_update": 0},
    "connection_count": 0,
    "last_command_time": time.time()
}

# ===== LiDAR Data Storage =====
lidar_points = collections.deque(maxlen=2000)  # Real-time point storage
lidar_lock = threading.Lock()
lidar_stop = threading.Event()

# ===== Utility Functions =====
def get_pi_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"

def stop_all_motors():
    if motors_ready:
        try:
            motor1_pwm.value = 0
            motor2_pwm.value = 0
            motor1_dir.off()
            motor2_dir.off()
            print("All motors stopped")
        except Exception as e:
            print(f"Error stopping motors: {e}")
    
    current_state["motors"] = {
        "motor1_speed": 0, "motor1_dir": "stop",
        "motor2_speed": 0, "motor2_dir": "stop"
    }

def control_motor(motor_num, speed, direction):
    if not motors_ready:
        return
    
    try:
        if motor_num == 1:  # Forward/Backward Motor
            if direction == "stop" or speed == 0:
                motor1_pwm.value = 0
                motor1_dir.off()
            elif direction == "forward":
                motor1_dir.on()
                motor1_pwm.value = abs(speed)
            elif direction == "backward":
                motor1_dir.off()
                motor1_pwm.value = abs(speed)
                
        elif motor_num == 2:  # Left/Right Motor
            if direction == "stop" or speed == 0:
                motor2_pwm.value = 0
                motor2_dir.off()
            elif direction == "right":
                motor2_dir.on()
                motor2_pwm.value = abs(speed)
            elif direction == "left":
                motor2_dir.off()
                motor2_pwm.value = abs(speed)
                
    except Exception as e:
        print(f"Motor {motor_num} control error: {e}")

def joystick_to_individual_motors(x, y):
    dead_zone = 0.1
    
    # Motor 1: Forward/Backward (Y-axis)
    if abs(y) < dead_zone:
        motor1_speed = 0
        motor1_dir = "stop"
    else:
        motor1_speed = abs(y)
        motor1_dir = "forward" if y > 0 else "backward"
    
    # Motor 2: Left/Right (X-axis)
    if abs(x) < dead_zone:
        motor2_speed = 0
        motor2_dir = "stop"
    else:
        motor2_speed = abs(x)
        motor2_dir = "right" if x > 0 else "left"
    
    return motor1_speed, motor1_dir, motor2_speed, motor2_dir

# ===== LiDAR Functions =====
def calc_crc8(data_bytes: bytes) -> int:
    crc = 0
    for b in data_bytes:
        crc = CRC_TABLE[(crc ^ b) & 0xff]
    return crc

def parse_frame_bytes(frame: bytes):
    try:
        ver_len = frame[1]
        speed, start_angle_raw = struct.unpack_from("<HH", frame, 2)
        offset = 6
        raw_points = []
        for i in range(POINTS_PER_PACK):
            dist, inten = struct.unpack_from("<HB", frame, offset)
            raw_points.append((dist, inten))
            offset += 3
        end_angle_raw, timestamp = struct.unpack_from("<HH", frame, offset)
    except Exception:
        return []

    start_deg = start_angle_raw / 100.0
    end_deg = end_angle_raw / 100.0
    if end_deg < start_deg:
        end_deg += 360.0
    step = (end_deg - start_deg) / (POINTS_PER_PACK - 1) if POINTS_PER_PACK > 1 else 0.0

    pts = []
    now = time.time()
    for i, (dist, inten) in enumerate(raw_points):
        if dist > 50:  # Filter out invalid readings
            # Fix upside-down orientation by flipping angle
            angle = (start_deg + step * i + 180.0) % 360.0
            pts.append({
                "timestamp": now,
                "angle": angle,
                "distance": dist,
                "intensity": inten
            })
    return pts

def lidar_reader_thread():
    """LiDAR data reading thread"""
    if LIDAR_SIMULATE:
        print("Starting LiDAR simulator")
        lidar_simulator()
        return
    
    try:
        ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=0.01)
        print(f"? LiDAR opened: {LIDAR_PORT}")
    except Exception as e:
        print(f"? LiDAR failed: {e}, starting simulator")
        lidar_simulator()
        return

    buffer = bytearray()
    try:
        while not lidar_stop.is_set():
            chunk = ser.read(4096)
            if chunk:
                buffer.extend(chunk)

            while True:
                idx = buffer.find(bytes([HEADER]))
                if idx == -1:
                    if len(buffer) > FRAME_LEN * 2:
                        del buffer[:-FRAME_LEN]
                    break
                
                if len(buffer) - idx < FRAME_LEN:
                    if idx > 0:
                        del buffer[:idx]
                    break
                
                frame = bytes(buffer[idx: idx + FRAME_LEN])
                crc_calc = calc_crc8(frame[:-1])
                crc_recv = frame[-1]
                
                if crc_calc == crc_recv:
                    parsed = parse_frame_bytes(frame)
                    if parsed:
                        with lidar_lock:
                            for p in parsed:
                                lidar_points.append(p)
                    del buffer[:idx + FRAME_LEN]
                else:
                    del buffer[idx]
            
            time.sleep(0.001)
            
    except Exception as e:
        print(f"LiDAR thread error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

def lidar_simulator():
    """LiDAR simulator for testing"""
    import random
    angle = 0
    while not lidar_stop.is_set():
        points = []
        now = time.time()
        
        # Generate a sweep of points
        for i in range(12):
            curr_angle = (angle + i * 5) % 360.0
            # Create some obstacles
            if 45 <= curr_angle <= 135:
                dist = 800 + random.randint(-50, 50)
            elif 225 <= curr_angle <= 315:
                dist = 1200 + random.randint(-100, 100)
            else:
                dist = 2000 + random.randint(-200, 200)
            
            points.append({
                "timestamp": now,
                "angle": curr_angle,
                "distance": dist,
                "intensity": random.randint(50, 200)
            })
        
        with lidar_lock:
            for p in points:
                lidar_points.append(p)
        
        angle = (angle + 15) % 360.0
        time.sleep(0.02)

# ===== Camera Functions =====
def get_camera_frame():
    """Get latest camera frame as base64 JPEG"""
    if not camera:
        return None
    
    try:
        # Capture frame
        frame = camera.capture_array()
        
        # Convert BGR to RGB for proper color display
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Encode as JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        # Convert to base64
        frame_b64 = base64.b64encode(buffer).decode('utf-8')
        return frame_b64
        
    except Exception as e:
        print(f"Camera capture error: {e}")
        return None

# ===== Flask Routes =====
@app.route('/')
def index():
    return render_template('integrated_index.html')

@app.route('/status')
def get_status():
    # Get latest LiDAR points
    with lidar_lock:
        recent_points = list(lidar_points)[-100:] if lidar_points else []
    
    return {
        "motors_ready": motors_ready,
        "camera_ready": camera is not None,
        "lidar_ready": True,
        "current_state": current_state,
        "pi_ip": get_pi_ip(),
        "lidar_points": recent_points[-50:]  # Send latest 50 points
    }

@app.route('/camera_feed')
def camera_feed():
    """Camera stream endpoint"""
    def generate_frames():
        while True:
            frame = get_camera_frame()
            if frame:
                yield f"data:image/jpeg;base64,{frame}\n\n"
            else:
                time.sleep(0.1)
    
    return Response(generate_frames(), mimetype='text/plain')

# ===== SocketIO Handlers =====
@socketio.on('connect')
def handle_connect():
    current_state["connection_count"] += 1
    client_ip = request.environ.get('REMOTE_ADDR', 'unknown')
    print(f"Client connected from {client_ip} (Total: {current_state['connection_count']})")
    
    emit('system_status', {
        "motors_ready": motors_ready,
        "camera_ready": camera is not None,
        "lidar_ready": True,
        "pi_ip": get_pi_ip()
    })

@socketio.on('disconnect')
def handle_disconnect():
    current_state["connection_count"] = max(0, current_state["connection_count"] - 1)
    print(f"Client disconnected (Remaining: {current_state['connection_count']})")
    
    if current_state["connection_count"] == 0:
        stop_all_motors()

@socketio.on('joystick_input')
def handle_joystick(data):
    try:
        x = float(data.get('x', 0))
        y = float(data.get('y', 0))
        
        current_state["joystick"] = {"x": x, "y": y, "active": abs(x) > 0.1 or abs(y) > 0.1}
        current_state["last_command_time"] = time.time()
        
        m1_speed, m1_dir, m2_speed, m2_dir = joystick_to_individual_motors(x, y)
        
        current_state["motors"] = {
            "motor1_speed": m1_speed, "motor1_dir": m1_dir,
            "motor2_speed": m2_speed, "motor2_dir": m2_dir
        }
        
        control_motor(1, m1_speed, m1_dir)
        control_motor(2, m2_speed, m2_dir)
        
        emit('motor_status_update', current_state["motors"], broadcast=True)
        
    except Exception as e:
        print(f"Joystick error: {e}")

@socketio.on('emergency_stop')
def handle_emergency_stop():
    print("EMERGENCY STOP!")
    stop_all_motors()
    current_state["joystick"] = {"x": 0, "y": 0, "active": False}
    socketio.emit('emergency_stop_activated', {"timestamp": time.time()}, broadcast=True)

@socketio.on('request_lidar_update')
def handle_lidar_request():
    """Send latest LiDAR data to client"""
    with lidar_lock:
        # Send recent points for real-time display
        recent_points = []
        now = time.time()
        for p in list(lidar_points)[-200:]:  # Latest 200 points
            if now - p["timestamp"] < 2.0:  # Points from last 2 seconds
                recent_points.append({
                    "angle": p["angle"],
                    "distance": p["distance"],
                    "intensity": p["intensity"],
                    "age": now - p["timestamp"]
                })
    
    emit('lidar_update', {"points": recent_points})

@socketio.on('request_camera_frame')
def handle_camera_request():
    """Send latest camera frame"""
    frame = get_camera_frame()
    if frame:
        emit('camera_frame', {"image": frame})

@socketio.on('gps_data')
def handle_gps_update(data):
    try:
        current_state["gps"] = {
            "lat": float(data.get('lat', 0)),
            "lon": float(data.get('lon', 0)),
            "accuracy": int(data.get('accuracy', 0)),
            "available": True
        }
        emit('gps_update', current_state["gps"], broadcast=True)
    except Exception as e:
        print(f"GPS error: {e}")

# ===== Background Threads =====
def data_broadcast_thread():
    """Continuously broadcast sensor data"""
    while True:
        try:
            if current_state["connection_count"] > 0:
                # Broadcast LiDAR data
                with socketio.test_request_context():
                    with lidar_lock:
                        if lidar_points:
                            recent_points = []
                            now = time.time()
                            for p in list(lidar_points)[-100:]:
                                if now - p["timestamp"] < 1.0:
                                    recent_points.append({
                                        "angle": p["angle"],
                                        "distance": p["distance"],
                                        "intensity": p["intensity"],
                                        "age": now - p["timestamp"]
                                    })
                            
                            if recent_points:
                                socketio.emit('lidar_update', {"points": recent_points})
                
                # Broadcast camera frame
                frame = get_camera_frame()
                if frame:
                    socketio.emit('camera_frame', {"image": frame})
            
            time.sleep(0.1)  # 10 FPS broadcast rate
            
        except Exception as e:
            print(f"Broadcast error: {e}")
            time.sleep(1)

def safety_monitor():
    while True:
        try:
            if (time.time() - current_state["last_command_time"] > 3.0 and 
                current_state["joystick"]["active"]):
                
                print("Auto-stop: No input for 3 seconds")
                stop_all_motors()
                current_state["joystick"]["active"] = False
                socketio.emit('auto_stop_triggered', {"reason": "timeout"})
            
            time.sleep(0.5)
            
        except Exception as e:
            print(f"Safety monitor error: {e}")
            time.sleep(1)

# ===== Main Entry Point =====
if __name__ == '__main__':
    try:
        # Start background threads
        lidar_thread = threading.Thread(target=lidar_reader_thread, daemon=True)
        lidar_thread.start()
        
        broadcast_thread = threading.Thread(target=data_broadcast_thread, daemon=True)
        broadcast_thread.start()
        
        safety_thread = threading.Thread(target=safety_monitor, daemon=True)
        safety_thread.start()
        
        pi_ip = get_pi_ip()
        print("\n" + "="*60)
        print("INTEGRATED MOTOR + LiDAR + CAMERA SERVER")
        print("="*60)
        print(f"? Access: http://{pi_ip}:5000")
        print(f"? Motors: {'? Ready' if motors_ready else '? Simulation'}")
        print(f"? Camera: {'? Ready' if camera else '? Not Available'}")
        print(f"? LiDAR: {'? Simulated' if LIDAR_SIMULATE else '? Hardware'}")
        print("="*60)
        
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
        
    except KeyboardInterrupt:
        print("\nShutdown requested")
        lidar_stop.set()
        stop_all_motors()
        if camera:
            camera.stop()
        
    except Exception as e:
        print(f"\nServer error: {e}")
        lidar_stop.set()
        stop_all_motors()
        if camera:
            camera.stop()
        
    finally:
        lidar_stop.set()
        if camera:
            try:
                camera.stop()
            except:
                pass
        if motors_ready:
            stop_all_motors()
