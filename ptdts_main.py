#!/usr/bin/env python3
"""
PTDTS - Pan Tilt Drone Detection and Tracking System
Main tracking engine with continuous motor control

Key Features:
- Picamera2 HQ camera at maximum FPS
- YOLO11n-NCNN drone detection
- Continuous motor control to center targets (NOT position-based)
- Web GUI with live video, YOLO overlay, and manual controls
- Real-time performance priority

Hardware:
- Raspberry Pi 5
- HQ Camera (Picamera2)
- 30:1 DC motor with encoder (8556 counts/360°)
- DRV8874 motor driver
"""

import cv2
import numpy as np
import time
import json
from threading import Thread, Lock, Event
from collections import deque
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import PWMOutputDevice, DigitalInputDevice
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO
import os

# ============================================================================
# CONFIGURATION
# ============================================================================

# Calibration constant (from calibration script)
COUNTS_PER_360 = -8556  # CW = negative

# GPIO Pin assignments
GPIO_ENCODER_A = 17
GPIO_ENCODER_B = 4
GPIO_MOTOR_IN1 = 27
GPIO_MOTOR_IN2 = 22

# Motor control parameters
MOTOR_PWM_FREQ = 1000  # Hz
MOTOR_MIN_SPEED = 0.30  # 30% minimum to overcome static friction
MOTOR_TRACKING_SPEED = 0.20  # 20% for smooth tracking once moving
MOTOR_MAX_SPEED = 0.60  # 60% maximum for safety
MOTOR_DEADZONE = 50  # pixels - increased from 20 to stop sooner

# Camera configuration
CAMERA_WIDTH = 1280  # Lower resolution for higher FPS
CAMERA_HEIGHT = 720
CAMERA_FPS = 60  # Target FPS (will use what camera can deliver)

# YOLO configuration
YOLO_MODEL_PATH = "models/yolov11n-UAV-finetune_ncnn_model"  # NCNN format for ARM64 performance
YOLO_CONF_THRESHOLD = 0.25  # Confidence threshold
YOLO_IOU_THRESHOLD = 0.45  # NMS IOU threshold
YOLO_CLASSES = None  # None = all classes, or specify list [0, 1, 2] for specific classes

# Web server configuration
WEB_PORT = 5000
JPEG_QUALITY = 80  # Balance quality vs latency

# ============================================================================
# QUADRATURE ENCODER CLASS
# ============================================================================

class QuadratureEncoder:
    """Thread-safe interrupt-driven quadrature encoder"""
    
    def __init__(self, pin_a, pin_b, counts_per_360):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counts_per_360 = counts_per_360
        self.count = 0
        self.lock = Lock()
        
        # Initialize GPIO
        self.encoder_a = DigitalInputDevice(pin_a, pull_up=True)
        self.encoder_b = DigitalInputDevice(pin_b, pull_up=True)
        
        # Attach interrupt handlers
        self.encoder_a.when_activated = self._on_a_rising
        self.encoder_a.when_deactivated = self._on_a_falling
        self.encoder_b.when_activated = self._on_b_rising
        self.encoder_b.when_deactivated = self._on_b_falling
        
        print(f"✓ Encoder initialized (GPIO {pin_a}, {pin_b})")
    
    def _on_a_rising(self):
        with self.lock:
            self.count += 1 if not self.encoder_b.value else -1
    
    def _on_a_falling(self):
        with self.lock:
            self.count += -1 if not self.encoder_b.value else 1
    
    def _on_b_rising(self):
        with self.lock:
            self.count += -1 if not self.encoder_a.value else 1
    
    def _on_b_falling(self):
        with self.lock:
            self.count += 1 if not self.encoder_a.value else -1
    
    def get_count(self):
        with self.lock:
            return self.count
    
    def reset(self):
        with self.lock:
            self.count = 0
    
    def get_angle(self):
        """Return angle in degrees (0-360)"""
        with self.lock:
            angle = (self.count / self.counts_per_360) * 360.0
            return angle % 360.0
    
    def cleanup(self):
        self.encoder_a.close()
        self.encoder_b.close()


# ============================================================================
# MOTOR DRIVER CLASS
# ============================================================================

class MotorDriver:
    """DRV8874 motor driver with smart speed control"""
    
    def __init__(self, pin_in1, pin_in2, pwm_freq=1000):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2
        
        # Initialize PWM outputs
        self.in1 = PWMOutputDevice(pin_in1, frequency=pwm_freq)
        self.in2 = PWMOutputDevice(pin_in2, frequency=pwm_freq)
        
        self.current_speed = 0.0
        self.is_moving = False
        self.lock = Lock()
        
        self.stop()
        print(f"✓ Motor driver initialized (GPIO {pin_in1}, {pin_in2})")
    
    def set_speed(self, speed):
        """
        Set motor speed with smart breakaway handling
        
        Args:
            speed: -1.0 to 1.0
                   Positive = CW (counts go negative)
                   Negative = CCW (counts go positive)
        """
        with self.lock:
            speed = max(-1.0, min(1.0, speed))
            
            # Apply minimum speed threshold when starting from stop
            if abs(speed) > 0 and not self.is_moving:
                # Apply breakaway threshold
                if abs(speed) < MOTOR_MIN_SPEED:
                    speed = MOTOR_MIN_SPEED if speed > 0 else -MOTOR_MIN_SPEED
                self.is_moving = True
            elif abs(speed) == 0:
                self.is_moving = False
            
            self.current_speed = speed
            
            if speed > 0:
                # Clockwise
                self.in1.value = abs(speed)
                self.in2.value = 0
            elif speed < 0:
                # Counter-clockwise
                self.in1.value = 0
                self.in2.value = abs(speed)
            else:
                # Stop
                self.in1.value = 0
                self.in2.value = 0
    
    def stop(self):
        with self.lock:
            self.current_speed = 0.0
            self.is_moving = False
            self.in1.value = 0
            self.in2.value = 0
    
    def get_speed(self):
        with self.lock:
            return self.current_speed
    
    def cleanup(self):
        self.stop()
        self.in1.close()
        self.in2.close()


# ============================================================================
# TRACKING CONTROLLER
# ============================================================================

class TrackingController:
    """
    Continuous tracking controller with PD control
    
    Key principle from plan.txt:
    - Motor receives CONTINUOUS input to center target
    - Encoder angle used for telemetry, NOT position control
    - Priority: Real-time tracking over precision
    """
    
    def __init__(self, motor, encoder):
        self.motor = motor
        self.encoder = encoder
        
        self.target_x = None  # Target center X coordinate
        self.frame_center_x = CAMERA_WIDTH / 2
        self.tracking_active = False
        
        # Control parameters
        self.deadzone = MOTOR_DEADZONE
        self.base_speed = MOTOR_TRACKING_SPEED
        self.max_speed = MOTOR_MAX_SPEED
        
        # PD control gains
        self.kp = 0.0008  # Proportional gain (from config tuning)
        self.kd = 0.0005  # Derivative gain (reduced from 0.0015 - less damping, more stable)
        
        # Derivative calculation with filtering
        self.last_error = 0.0
        self.last_time = time.time()
        self.filtered_derivative = 0.0  # Low-pass filtered derivative
        self.derivative_alpha = 0.3  # Filter coefficient (0=no filtering, 1=no smoothing)
        self.max_derivative = 500.0  # Maximum allowed derivative (px/s)
        
        # Debug logging
        self.debug_mode = True
        self.debug_counter = 0
        
        self.lock = Lock()
        print("✓ Tracking controller initialized (PD control with filtering)")
    
    def update_target(self, target_center_x):
        """Update target position (called from detection thread)"""
        with self.lock:
            self.target_x = target_center_x
            self.tracking_active = True
    
    def clear_target(self):
        """Clear target (no detection)"""
        with self.lock:
            self.target_x = None
            self.tracking_active = False
    
    def compute_motor_command(self):
        """
        Compute continuous motor command using filtered PD control
        
        P term: Proportional to error (how far off target)
        D term: Proportional to filtered rate of change (damping)
        
        Returns:
            float: Motor speed (-1.0 to 1.0)
        """
        with self.lock:
            if not self.tracking_active or self.target_x is None:
                self.last_error = 0.0
                self.filtered_derivative = 0.0
                self.last_time = time.time()
                return 0.0
            
            # Calculate error (inverted: target to right needs CCW, to left needs CW)
            error = self.frame_center_x - self.target_x
            
            # Apply deadzone
            if abs(error) < self.deadzone:
                self.last_error = 0.0
                self.filtered_derivative = 0.0
                self.last_time = time.time()
                return 0.0
            
            # Calculate derivative (rate of change of error)
            current_time = time.time()
            dt = current_time - self.last_time
            raw_derivative = 0.0  # Initialize for debug logging
            
            # Prevent division by very small dt (< 5ms is unrealistic)
            if dt > 0.005:
                raw_derivative = (error - self.last_error) / dt
                
                # Limit derivative to prevent spikes from YOLO jitter
                raw_derivative = max(-self.max_derivative, 
                                   min(self.max_derivative, raw_derivative))
                
                # Apply low-pass filter (exponential moving average)
                # This smooths out sudden spikes from detection noise
                self.filtered_derivative = (self.derivative_alpha * raw_derivative + 
                                          (1 - self.derivative_alpha) * self.filtered_derivative)
            else:
                # dt too small - can't calculate fresh derivative
                # CRITICAL: Decay the filtered derivative toward zero to prevent windup
                decay_factor = 0.9  # Decay by 10% each iteration when no update
                self.filtered_derivative *= decay_factor
            
            # PD control formula with filtered derivative
            # P term makes it move toward target
            # Filtered D term dampens the motion smoothly
            speed = (self.kp * error) + (self.kd * self.filtered_derivative)
            
            # Update last values for next iteration
            self.last_error = error
            self.last_time = current_time
            
            # Apply base speed threshold and max speed limit
            if abs(speed) < self.base_speed and abs(speed) > 0:
                speed = self.base_speed if speed > 0 else -self.base_speed
            
            speed = max(-self.max_speed, min(self.max_speed, speed))
            
            # Debug logging (print every 10th iteration to avoid spam)
            if self.debug_mode:
                self.debug_counter += 1
                if self.debug_counter % 10 == 0:
                    p_term = self.kp * error
                    d_term = self.kd * self.filtered_derivative
                    print(f"[TRACK] err={error:6.1f}px | raw_d={raw_derivative:7.1f} | filt_d={self.filtered_derivative:7.1f} | P={p_term:+.3f} | D={d_term:+.3f} | speed={speed:+.3f}")
            
            return speed
    
    def run(self):
        """
        Continuous control loop
        Should run in separate thread at high rate
        """
        print("✓ Tracking control loop started")
        
        while True:
            motor_cmd = self.compute_motor_command()
            self.motor.set_speed(motor_cmd)
            time.sleep(0.01)  # 100 Hz control loop


# ============================================================================
# CAMERA & DETECTION PIPELINE
# ============================================================================

class DetectionPipeline:
    """Camera capture + YOLO detection pipeline"""
    
    def __init__(self, model_path):
        # Initialize camera
        print("Initializing camera...")
        self.picam2 = Picamera2(1)
        config = self.picam2.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)  # Camera warm-up
        print(f"✓ Camera initialized ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
        
        # Initialize YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        print("✓ YOLO model loaded")
        
        # Frame buffer for web streaming
        self.current_frame = None
        self.frame_lock = Lock()
        
        # Detection results
        self.detections = []
        self.detection_lock = Lock()
        
        # Performance metrics
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
    
    def capture_and_detect(self):
        """Capture frame and run YOLO detection"""
        # Capture frame
        frame = self.picam2.capture_array()
        
        # Run YOLO detection
        results = self.model(
            frame,
            conf=YOLO_CONF_THRESHOLD,
            iou=YOLO_IOU_THRESHOLD,
            classes=YOLO_CLASSES,
            verbose=False
        )
        
        # Parse detections
        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                # Calculate center
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center': [int(center_x), int(center_y)],
                    'confidence': conf,
                    'class': cls
                })
        
        # Update FPS counter
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = current_time
        
        return frame, detections
    
    def draw_overlay(self, frame, detections, encoder_angle, motor_speed):
        """Draw YOLO boxes, crosshair, and telemetry overlay"""
        overlay = frame.copy()
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # Draw center crosshair
        cv2.drawMarker(overlay, (center_x, center_y), (0, 255, 0), 
                      cv2.MARKER_CROSS, 30, 2)
        
        # Draw detections and tracking lines
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cx, cy = det['center']
            conf = det['confidence']
            
            # Draw bounding box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
            
            # Draw center point
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
            
            # Draw line from frame center to detection center
            cv2.line(overlay, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
            
            # Draw label
            label = f"Drone {conf:.2f}"
            cv2.putText(overlay, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw telemetry overlay
        telemetry = [
            f"FPS: {self.fps}",
            f"Angle: {encoder_angle:.1f}°",
            f"Motor: {motor_speed:+.0%}",
            f"Detections: {len(detections)}"
        ]
        
        y_offset = 30
        for text in telemetry:
            cv2.putText(overlay, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 30
        
        return overlay
    
    def get_current_frame(self):
        """Get current frame with overlay (for web streaming)"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            return None
    
    def get_detections(self):
        """Get current detections"""
        with self.detection_lock:
            return self.detections.copy()
    
    def run(self, tracking_controller, encoder, motor):
        """
        Main detection loop
        Runs in separate thread
        """
        print("✓ Detection pipeline started")
        
        while True:
            # Capture and detect
            frame, detections = self.capture_and_detect()
            
            # Update tracking controller with highest-confidence detection
            if len(detections) > 0:
                # Sort by confidence, take highest
                detections.sort(key=lambda d: d['confidence'], reverse=True)
                best_detection = detections[0]
                tracking_controller.update_target(best_detection['center'][0])
            else:
                tracking_controller.clear_target()
            
            # Draw overlay
            encoder_angle = encoder.get_angle()
            motor_speed = motor.get_speed()
            overlay_frame = self.draw_overlay(frame, detections, encoder_angle, motor_speed)
            
            # Update frame buffer
            with self.frame_lock:
                self.current_frame = overlay_frame
            
            with self.detection_lock:
                self.detections = detections


# ============================================================================
# WEB SERVER
# ============================================================================

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global references (will be set in main)
detection_pipeline = None
tracking_controller = None
encoder = None
motor = None

@app.route('/')
def index():
    """Serve main web interface"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            if detection_pipeline is None:
                time.sleep(0.1)
                continue
            
            frame = detection_pipeline.get_current_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', frame, 
                                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    """Get system status"""
    return jsonify({
        'encoder_angle': encoder.get_angle() if encoder else 0,
        'encoder_count': encoder.get_count() if encoder else 0,
        'motor_speed': motor.get_speed() if motor else 0,
        'detections': detection_pipeline.get_detections() if detection_pipeline else [],
        'tracking_active': tracking_controller.tracking_active if tracking_controller else False
    })

@app.route('/api/manual_control', methods=['POST'])
def api_manual_control():
    """Manual motor control override"""
    data = request.json
    command = data.get('command', '')
    
    if command == 'stop':
        motor.stop()
    elif command == 'cw':
        speed = data.get('speed', 0.3)
        motor.set_speed(speed)
    elif command == 'ccw':
        speed = data.get('speed', 0.3)
        motor.set_speed(-speed)
    elif command == 'reset_encoder':
        encoder.reset()
    
    return jsonify({'status': 'ok'})


# ============================================================================
# MAIN
# ============================================================================

def main():
    global detection_pipeline, tracking_controller, encoder, motor
    
    print("=" * 70)
    print("PTDTS - PAN TILT DRONE DETECTION & TRACKING SYSTEM")
    print("=" * 70)
    print("\nInitializing hardware...")
    
    # Initialize hardware
    encoder = QuadratureEncoder(GPIO_ENCODER_A, GPIO_ENCODER_B, COUNTS_PER_360)
    motor = MotorDriver(GPIO_MOTOR_IN1, GPIO_MOTOR_IN2, MOTOR_PWM_FREQ)
    
    # Initialize tracking controller
    tracking_controller = TrackingController(motor, encoder)
    
    # Initialize detection pipeline
    detection_pipeline = DetectionPipeline(YOLO_MODEL_PATH)
    
    print("\n" + "=" * 70)
    print("Starting system threads...")
    print("=" * 70)
    
    # Start tracking control loop
    tracking_thread = Thread(target=tracking_controller.run, daemon=True)
    tracking_thread.start()
    
    # Start detection pipeline
    detection_thread = Thread(
        target=detection_pipeline.run,
        args=(tracking_controller, encoder, motor),
        daemon=True
    )
    detection_thread.start()
    
    print("\n✓ All systems running!")
    print(f"\n🌐 Web GUI available at: http://localhost:{WEB_PORT}")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        # Start web server
        socketio.run(app, host='0.0.0.0', port=WEB_PORT, debug=False)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    finally:
        motor.cleanup()
        encoder.cleanup()
        detection_pipeline.picam2.stop()
        print("✓ Cleanup complete")


if __name__ == "__main__":
    main()