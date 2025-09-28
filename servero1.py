#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Motor Control Server with Individual Motor Control
Motor 1: Forward/Backward control (Y-axis)
Motor 2: Left/Right control (X-axis)
"""

from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time
import threading
import socket

# ===== Motor Driver Pin Configuration =====
# Motor 1 (Forward/Backward): DIR1=GPIO18, PWM1=GPIO19
# Motor 2 (Left/Right): DIR2=GPIO20, PWM2=GPIO21
DIR1_PIN = 18   # Motor 1 Direction (Forward/Backward)
PWM1_PIN = 19   # Motor 1 Speed (Forward/Backward)
DIR2_PIN = 20   # Motor 2 Direction (Left/Right)
PWM2_PIN = 21   # Motor 2 Speed (Left/Right)

print("Individual Motor Control Configuration:")
print("   Motor 1 (Y-axis - Forward/Back): DIR={}, PWM={}".format(DIR1_PIN, PWM1_PIN))
print("   Motor 2 (X-axis - Left/Right): DIR={}, PWM={}".format(DIR2_PIN, PWM2_PIN))

# ===== Initialize Motors =====
motors_ready = False

try:
    # Direction control (Digital pins)
    motor1_dir = DigitalOutputDevice(DIR1_PIN)  # Forward/Backward
    motor2_dir = DigitalOutputDevice(DIR2_PIN)  # Left/Right
    
    # Speed control (PWM pins)
    motor1_pwm = PWMOutputDevice(PWM1_PIN, frequency=1000)  # Forward/Backward
    motor2_pwm = PWMOutputDevice(PWM2_PIN, frequency=1000)  # Left/Right
    
    motors_ready = True
    print("Motors initialized successfully!")
    
except Exception as e:
    print("Motor initialization failed: {}".format(e))
    print("Running in simulation mode")
    motors_ready = False

# ===== Flask Setup =====
app = Flask(__name__)
app.config['SECRET_KEY'] = 'individual_motor_control_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ===== Global State =====
current_state = {
    "joystick": {"x": 0, "y": 0, "active": False},
    "motors": {
        "motor1_speed": 0, "motor1_dir": "stop",  # Forward/Backward
        "motor2_speed": 0, "motor2_dir": "stop"   # Left/Right
    },
    "gps": {"lat": 0, "lon": 0, "accuracy": 0, "available": False},
    "connection_count": 0,
    "last_command_time": time.time()
}

def get_pi_ip():
    """Get Raspberry Pi IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"

def stop_all_motors():
    """Stop both motors immediately"""
    if motors_ready:
        try:
            motor1_pwm.value = 0  # Stop Forward/Backward motor
            motor2_pwm.value = 0  # Stop Left/Right motor
            motor1_dir.off()
            motor2_dir.off()
            print("All motors stopped")
        except Exception as e:
            print("Error stopping motors: {}".format(e))
    
    current_state["motors"] = {
        "motor1_speed": 0, "motor1_dir": "stop",
        "motor2_speed": 0, "motor2_dir": "stop"
    }

def control_motor(motor_num, speed, direction):
    """Control individual motor"""
    if not motors_ready:
        return
    
    try:
        if motor_num == 1:  # Forward/Backward Motor
            if direction == "stop" or speed == 0:
                motor1_pwm.value = 0
                motor1_dir.off()
            elif direction == "forward":
                motor1_dir.on()   # DIR1 HIGH = Forward
                motor1_pwm.value = abs(speed)
            elif direction == "backward":
                motor1_dir.off()  # DIR1 LOW = Backward
                motor1_pwm.value = abs(speed)
                
        elif motor_num == 2:  # Left/Right Motor
            if direction == "stop" or speed == 0:
                motor2_pwm.value = 0
                motor2_dir.off()
            elif direction == "right":
                motor2_dir.on()   # DIR2 HIGH = Right
                motor2_pwm.value = abs(speed)
            elif direction == "left":
                motor2_dir.off()  # DIR2 LOW = Left
                motor2_pwm.value = abs(speed)
                
    except Exception as e:
        print("Motor {} control error: {}".format(motor_num, e))

def joystick_to_individual_motors(x, y):
    """
    Convert joystick to individual motor control
    Y-axis (-1 to +1): Motor 1 Forward/Backward
    X-axis (-1 to +1): Motor 2 Left/Right
    """
    
    # Dead zone - ignore small movements
    dead_zone = 0.1
    
    # Motor 1: Forward/Backward (Y-axis control)
    if abs(y) < dead_zone:
        motor1_speed = 0
        motor1_dir = "stop"
    else:
        motor1_speed = abs(y)
        motor1_dir = "forward" if y > 0 else "backward"
    
    # Motor 2: Left/Right (X-axis control)  
    if abs(x) < dead_zone:
        motor2_speed = 0
        motor2_dir = "stop"
    else:
        motor2_speed = abs(x)
        motor2_dir = "right" if x > 0 else "left"
    
    return motor1_speed, motor1_dir, motor2_speed, motor2_dir

# ===== Flask Routes =====
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/status')
def get_status():
    return {
        "motors_ready": motors_ready,
        "current_state": current_state,
        "pi_ip": get_pi_ip(),
        "control_mode": "individual",
        "motor1_function": "forward_backward",
        "motor2_function": "left_right"
    }

# ===== SocketIO Handlers =====
@socketio.on('connect')
def handle_connect():
    current_state["connection_count"] += 1
    client_ip = request.environ.get('REMOTE_ADDR', 'unknown')
    print("Client connected from {} (Total: {})".format(client_ip, current_state["connection_count"]))
    
    emit('system_status', {
        "motors_ready": motors_ready,
        "pi_ip": get_pi_ip(),
        "control_mode": "individual",
        "motor1_function": "Forward/Backward (Y-axis)",
        "motor2_function": "Left/Right (X-axis)"
    })

@socketio.on('disconnect')
def handle_disconnect():
    current_state["connection_count"] = max(0, current_state["connection_count"] - 1)
    print("Client disconnected (Remaining: {})".format(current_state["connection_count"]))
    
    if current_state["connection_count"] == 0:
        print("All clients disconnected - stopping motors")
        stop_all_motors()
        socketio.emit('motors_stopped', {"reason": "all_clients_disconnected"})

@socketio.on('joystick_input')
def handle_joystick(data):
    try:
        x = float(data.get('x', 0))  # Left/Right control
        y = float(data.get('y', 0))  # Forward/Backward control
        
        # Update state
        current_state["joystick"] = {"x": x, "y": y, "active": abs(x) > 0.1 or abs(y) > 0.1}
        current_state["last_command_time"] = time.time()
        
        # Convert to individual motor commands
        m1_speed, m1_dir, m2_speed, m2_dir = joystick_to_individual_motors(x, y)
        
        # Update motor state
        current_state["motors"] = {
            "motor1_speed": m1_speed, "motor1_dir": m1_dir,  # Forward/Backward
            "motor2_speed": m2_speed, "motor2_dir": m2_dir   # Left/Right
        }
        
        # Control physical motors
        control_motor(1, m1_speed, m1_dir)  # Forward/Backward motor
        control_motor(2, m2_speed, m2_dir)  # Left/Right motor
        
        # Send status to all clients
        emit('motor_status_update', current_state["motors"], broadcast=True)
        
        # Debug output
        if current_state["joystick"]["active"]:
            print("Joystick: X={:+.2f} Y={:+.2f} -> M1(F/B): {:.2f}({}) | M2(L/R): {:.2f}({})".format(
                x, y, m1_speed, m1_dir, m2_speed, m2_dir))
        
    except Exception as e:
        print("Joystick error: {}".format(e))
        emit('error', {"message": "Joystick input error"})

@socketio.on('emergency_stop')
def handle_emergency_stop():
    print("EMERGENCY STOP!")
    stop_all_motors()
    current_state["joystick"] = {"x": 0, "y": 0, "active": False}
    socketio.emit('emergency_stop_activated', {"timestamp": time.time()}, broadcast=True)

@socketio.on('gps_data')
def handle_gps_update(data):
    try:
        current_state["gps"] = {
            "lat": float(data.get('lat', 0)),
            "lon": float(data.get('lon', 0)),
            "accuracy": int(data.get('accuracy', 0)),
            "available": True
        }
        
        print("GPS: {:.6f}, {:.6f} (+/-{}m)".format(
            current_state['gps']['lat'], current_state['gps']['lon'], 
            current_state['gps']['accuracy']))
        
        emit('gps_update', current_state["gps"], broadcast=True)
        
    except Exception as e:
        print("GPS error: {}".format(e))

# ===== Safety Monitor =====
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
            print("Safety monitor error: {}".format(e))
            time.sleep(1)

# Start safety monitor
safety_thread = threading.Thread(target=safety_monitor, daemon=True)
safety_thread.start()

# ===== Main Entry Point =====
if __name__ == '__main__':
    try:
        pi_ip = get_pi_ip()
        print("\n" + "="*50)
        print("INDIVIDUAL MOTOR CONTROL SERVER")
        print("="*50)
        print("Raspberry Pi IP: {}".format(pi_ip))
        print("Web Interface: http://{}:5000".format(pi_ip))
        print("Motors Ready: {}".format(motors_ready))
        print("")
        print("CONTROL MAPPING:")
        print("  Y-axis (Up/Down)    -> Motor 1 (Forward/Backward)")
        print("  X-axis (Left/Right) -> Motor 2 (Left/Right)")
        print("="*50)
        
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
        
    except KeyboardInterrupt:
        print("\nShutdown requested")
        stop_all_motors()
        
    except Exception as e:
        print("\nServer error: {}".format(e))
        stop_all_motors()
        
    finally:
        if motors_ready:
            stop_all_motors()
