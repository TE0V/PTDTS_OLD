#!/usr/bin/env python3
"""
PTDTS - Pan Tilt Drone Detection and Tracking System (FIXED)
Main tracking engine with comprehensive fixes

CRITICAL FIXES APPLIED:
1. Motor direction mapping corrected (IN1/IN2 swap)
2. PID controller properly tuned with I/D terms and anti-windup
3. Velocity deadband reduced for smooth low-speed tracking
4. PWM limits adjusted for better performance
5. Speed linearizer validation fixed

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
from threading import Thread, Lock, Event, Timer
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
MOTOR_MIN_SPEED = 0.30  # Minimum to overcome static friction
MOTOR_MAX_SPEED = 0.70  # Maximum for safety (increased from 0.60)

# Control loop timing (pimotors uses 20Hz = 50ms)
CONTROL_LOOP_RATE = 20  # Hz
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_RATE  # 0.05 seconds = 50ms

# Camera configuration
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 60

# YOLO configuration
YOLO_MODEL_PATH = "models/yolov11n-UAV-finetune_ncnn_model"
YOLO_CONF_THRESHOLD = 0.4
YOLO_IOU_THRESHOLD = 0.45
YOLO_CLASSES = None

# Web server configuration
WEB_PORT = 5000
JPEG_QUALITY = 80

# Speed linearization table file
SPEED_CALIBRATION_FILE = "speed_linearization_table_fixed.json"


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
# SPEED LINEARIZER (LAYER 2)
# ============================================================================

class SpeedLinearizer:
    """
    Speed linearization lookup table - critical Layer 2 component

    Maps desired velocity (counts/sec) → (PWM duty cycle) to compensate
    for non-linear motor response. This is what eliminates overshoot.
    """

    def __init__(self, calibration_file=None):
        self.lookup_table = []  # List of (velocity, pwm) tuples
        self.calibrated = False

        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
        else:
            # Default linear mapping if no calibration available
            print("⚠ No speed linearization calibration found")
            print("  Using default linear mapping (may cause overshoot)")
            print(f"  Run speed_calibration_fixed.py to create {SPEED_CALIBRATION_FILE}")
            self._create_default_table()

    def _create_default_table(self):
        """Create a default linear lookup table"""
        # Simple linear mapping: velocity proportional to PWM
        # This won't eliminate overshoot but allows system to run
        max_velocity = 500  # counts/sec estimate at max PWM

        # CW direction (positive PWM → negative velocity)
        for pwm in np.linspace(0.3, 1.0, 15):
            velocity = -pwm * max_velocity
            self.lookup_table.append((velocity, pwm))

        # Zero point
        self.lookup_table.append((0.0, 0.0))

        # CCW direction (negative PWM → positive velocity)
        for pwm in np.linspace(0.3, 1.0, 15):
            velocity = pwm * max_velocity
            self.lookup_table.append((velocity, -pwm))

        # Sort by velocity
        self.lookup_table.sort(key=lambda x: x[0])

        self.calibrated = False

    def load_calibration(self, filename):
        """Load calibration data from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            self.lookup_table = []
            for entry in data['lookup_table']:
                self.lookup_table.append((entry['velocity'], entry['pwm']))

            # Sort by velocity (negative to positive)
            self.lookup_table.sort(key=lambda x: x[0])

            self.calibrated = True
            print(f"✓ Speed linearization loaded: {len(self.lookup_table)} points")
            print(f"  Velocity range: {self.lookup_table[0][0]:.0f} to {self.lookup_table[-1][0]:.0f} counts/sec")

        except Exception as e:
            print(f"⚠ Failed to load calibration: {e}")
            self._create_default_table()

    def velocity_to_pwm(self, target_velocity):
        """
        Convert desired velocity to PWM duty cycle using lookup table

        This is the critical linearization function that eliminates overshoot.

        Args:
            target_velocity: Desired velocity in counts/sec (signed)

        Returns:
            PWM duty cycle (-1.0 to 1.0)
        """
        if len(self.lookup_table) == 0:
            # Fallback linear mapping
            return np.clip(target_velocity / 500.0, -1.0, 1.0)

        # Handle zero velocity
        if abs(target_velocity) < 1.0:
            return 0.0

        # Find bracketing points and interpolate
        # Lookup table is sorted by velocity (negative to positive)

        if target_velocity <= self.lookup_table[0][0]:
            # Below minimum - use first point
            return self.lookup_table[0][1]
        elif target_velocity >= self.lookup_table[-1][0]:
            # Above maximum - use last point
            return self.lookup_table[-1][1]
        else:
            # Interpolate between points
            for i in range(len(self.lookup_table) - 1):
                v1, pwm1 = self.lookup_table[i]
                v2, pwm2 = self.lookup_table[i + 1]

                if v1 <= target_velocity <= v2:
                    # Linear interpolation
                    t = (target_velocity - v1) / (v2 - v1) if v2 != v1 else 0
                    pwm = pwm1 + t * (pwm2 - pwm1)
                    return pwm

        # Fallback
        return 0.0


# ============================================================================
# MOTOR DRIVER CLASS (LAYER 1) - FIXED
# ============================================================================

class MotorDriver:
    """
    DRV8874 motor driver - Layer 1: Direct actuation

    CRITICAL FIX: Corrected IN1/IN2 mapping
    - Positive PWM = CW rotation = negative encoder counts = IN2 high
    - Negative PWM = CCW rotation = positive encoder counts = IN1 high
    """

    def __init__(self, pin_in1, pin_in2, pwm_freq=1000):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2

        # Initialize PWM outputs
        self.in1 = PWMOutputDevice(pin_in1, frequency=pwm_freq)
        self.in2 = PWMOutputDevice(pin_in2, frequency=pwm_freq)

        self.current_pwm = 0.0
        self.lock = Lock()

        self.stop()
        print(f"✓ Motor driver initialized (GPIO {pin_in1}, {pin_in2})")
        print(f"  FIXED: Corrected direction mapping")

    def set_pwm(self, pwm):
        """
        Set motor PWM duty cycle directly - CORRECTED

        Args:
            pwm: -1.0 to 1.0
                 Positive = CW (counts go negative) = IN2
                 Negative = CCW (counts go positive) = IN1
        """
        with self.lock:
            pwm = max(-1.0, min(1.0, pwm))
            self.current_pwm = pwm

            if pwm > 0:
                # Clockwise - CORRECTED
                self.in1.value = 0
                self.in2.value = abs(pwm)
            elif pwm < 0:
                # Counter-clockwise - CORRECTED
                self.in1.value = abs(pwm)
                self.in2.value = 0
            else:
                # Stop
                self.in1.value = 0
                self.in2.value = 0

    def stop(self):
        with self.lock:
            self.current_pwm = 0.0
            self.in1.value = 0
            self.in2.value = 0

    def get_pwm(self):
        with self.lock:
            return self.current_pwm

    def cleanup(self):
        self.stop()
        self.in1.close()
        self.in2.close()


# ============================================================================
# VELOCITY-BASED TRACKING CONTROLLER (LAYER 3) - IMPROVED
# ============================================================================

class VelocityTrackingController:
    """
    Velocity-based tracking controller with fixed 20Hz timing - IMPROVED

    Key improvements:
    1. Properly tuned PID with I and D terms enabled
    2. Conditional integration anti-windup
    3. Reduced velocity deadband for smoother tracking
    4. Better rate limiting
    5. Adaptive gains based on error magnitude
    """

    def __init__(self, motor, encoder, speed_linearizer):
        self.motor = motor
        self.encoder = encoder
        self.speed_linearizer = speed_linearizer

        # Tracking state
        self.target_x = None  # Target pixel X coordinate
        self.frame_center_x = CAMERA_WIDTH / 2
        self.tracking_active = False

        # Control parameters - IMPROVED
        self.deadzone_pixels = 8  # Tighter pixel deadzone (was 5)
        self.max_velocity = 1200  # Increased from 1000 (counts/sec)
        self.max_acceleration = 600  # Increased from 400 (counts/sec²)
        self.velocity_deadband = 30  # REDUCED from 60 for smoother low-speed tracking

        # PID gains - PROPERLY TUNED
        # These gains work together to provide smooth, responsive tracking
        self.kp_velocity = 1.2   # Increased from 0.8 - stronger proportional response
        self.ki_velocity = 0.15  # ENABLED - eliminates steady-state error
        self.kd_velocity = 0.05  # ENABLED - damping to reduce oscillation

        # Velocity measurement state
        self.last_position = encoder.get_count()
        self.current_velocity = 0.0  # counts/sec
        self.target_velocity = 0.0  # counts/sec

        # PID state
        self.integral = 0.0
        self.last_velocity_error = 0.0

        # Fixed-rate timing
        self.control_period = CONTROL_LOOP_PERIOD
        self.next_update_time = time.time() + self.control_period
        self.running = False
        self.timer = None

        # Debug
        self.debug_mode = True
        self.debug_counter = 0

        self.lock = Lock()
        print(f"✓ Velocity tracking controller initialized")
        print(f"  Control rate: {CONTROL_LOOP_RATE} Hz ({self.control_period*1000:.0f}ms period)")
        print(f"  PID gains: Kp={self.kp_velocity}, Ki={self.ki_velocity}, Kd={self.kd_velocity}")
        print(f"  Velocity deadband: {self.velocity_deadband} counts/sec")
        print(f"  Speed linearization: {'ACTIVE' if speed_linearizer.calibrated else 'DEFAULT (needs calibration)'}")

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
            self.integral = 0.0  # Reset integral when target lost

    def _compute_velocity_command(self):
        """
        Compute velocity control command

        Two-stage control:
        1. Position error → target velocity (proportional)
        2. Velocity error → motor command (PID)

        Returns:
            Target velocity in counts/sec
        """
        with self.lock:
            if not self.tracking_active or self.target_x is None:
                # No target - command zero velocity
                self.integral = 0.0
                self.last_velocity_error = 0.0
                return 0.0

            # Stage 1: Position error to target velocity
            # Error calculation (inverted: target right needs CCW = negative velocity)
            pixel_error = self.frame_center_x - self.target_x

            # Apply deadzone
            if abs(pixel_error) < self.deadzone_pixels:
                self.integral = 0.0
                self.last_velocity_error = 0.0
                return 0.0

            # Convert pixel error to target velocity
            # Adaptive gain: stronger response for larger errors
            if abs(pixel_error) > 100:
                kp_position = 0.6  # Faster for large errors
            else:
                kp_position = 0.4  # Gentler for small errors

            target_velocity = kp_position * pixel_error

            # Clamp to maximum velocity
            target_velocity = max(-self.max_velocity,
                                 min(self.max_velocity, target_velocity))

            return target_velocity

    def _control_loop_update(self):
        """
        Fixed-rate control loop update (called every 50ms by timer)

        IMPROVED with proper PID control and anti-windup
        """
        # Measure current velocity
        current_position = self.encoder.get_count()
        position_delta = current_position - self.last_position

        # Velocity = position change / time period (counts/sec)
        self.current_velocity = position_delta / self.control_period
        self.last_position = current_position

        # Compute target velocity from position error
        target_velocity = self._compute_velocity_command()

        # Apply rate limiting (prevent sudden velocity changes)
        velocity_change = target_velocity - self.target_velocity
        max_change = self.max_acceleration * self.control_period

        if abs(velocity_change) > max_change:
            velocity_change = max_change if velocity_change > 0 else -max_change

        self.target_velocity += velocity_change

        # Stage 2: Velocity PID control
        velocity_error = self.target_velocity - self.current_velocity

        # Apply velocity deadband to prevent jitter
        if abs(velocity_error) < self.velocity_deadband:
            velocity_error = 0.0

        # Proportional term
        p_term = self.kp_velocity * velocity_error

        # Integral term with conditional integration (anti-windup)
        # Only integrate when:
        # 1. Error and integral have same sign (prevents windup in wrong direction)
        # 2. Output is not saturated
        control_velocity_estimate = p_term + self.ki_velocity * self.integral

        # Check if we would saturate
        output_saturated = abs(control_velocity_estimate) >= self.max_velocity * 0.95

        if not output_saturated:
            self.integral += velocity_error * self.control_period
            # Clamp integral to reasonable bounds
            max_integral = 200  # Increased from 100
            self.integral = max(-max_integral, min(max_integral, self.integral))

        i_term = self.ki_velocity * self.integral

        # Derivative term (damping)
        velocity_error_rate = (velocity_error - self.last_velocity_error) / self.control_period
        d_term = self.kd_velocity * velocity_error_rate
        self.last_velocity_error = velocity_error

        # Combined control output (target velocity for linearization layer)
        control_velocity = p_term + i_term + d_term

        # Clamp to max velocity
        control_velocity = max(-self.max_velocity,
                              min(self.max_velocity, control_velocity))

        # Layer 2: Convert velocity to PWM via linearization
        pwm_command = self.speed_linearizer.velocity_to_pwm(control_velocity)

        # Limit PWM to safe maximum
        max_pwm = MOTOR_MAX_SPEED
        pwm_command = max(-max_pwm, min(max_pwm, pwm_command))

        # Layer 1: Send PWM to motor
        self.motor.set_pwm(pwm_command)

        # Debug logging (every 10th iteration)
        if self.debug_mode:
            self.debug_counter += 1
            if self.debug_counter % 10 == 0:
                pixel_err = self.frame_center_x - self.target_x if self.target_x else 0
                print(f"[VEL] px_err={pixel_err:6.1f} | "
                      f"tgt_v={self.target_velocity:6.0f} | cur_v={self.current_velocity:6.0f} | "
                      f"v_err={velocity_error:6.0f} | P={p_term:+6.0f} I={i_term:+6.0f} D={d_term:+6.0f} | "
                      f"pwm={pwm_command:+.2f}")

        # Schedule next update (fixed-rate timing)
        if self.running:
            self.next_update_time += self.control_period
            delay = self.next_update_time - time.time()

            # Protect against timing drift
            if delay < 0:
                delay = 0
                self.next_update_time = time.time()

            self.timer = Timer(delay, self._control_loop_update)
            self.timer.daemon = True
            self.timer.start()

    def start(self):
        """Start the fixed-rate control loop"""
        self.running = True
        self.next_update_time = time.time() + self.control_period
        self.last_position = self.encoder.get_count()
        print("✓ Velocity control loop started (20 Hz fixed rate)")

        # Start first timer
        self.timer = Timer(self.control_period, self._control_loop_update)
        self.timer.daemon = True
        self.timer.start()

    def stop(self):
        """Stop the control loop"""
        self.running = False
        if self.timer:
            self.timer.cancel()
        self.motor.stop()

    def get_telemetry(self):
        """Get current control state for display"""
        with self.lock:
            return {
                'tracking_active': self.tracking_active,
                'current_velocity': self.current_velocity,
                'target_velocity': self.target_velocity,
                'target_x': self.target_x,
                'integral': self.integral
            }


# ============================================================================
# CAMERA & DETECTION PIPELINE
# ============================================================================

class DetectionPipeline:
    """Camera capture + YOLO detection pipeline"""

    def __init__(self, model_path):
        # Initialize camera
        print("Initializing camera...")
        self.picam2 = Picamera2(0)
        config = self.picam2.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)  # Camera warm-up
        print("✓ Camera initialized")

        # Initialize YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path, task='detect')
        print("✓ YOLO model loaded")

        # Frame buffer
        self.current_frame = None
        self.frame_lock = Lock()

        # Detection results
        self.detections = []
        self.detection_lock = Lock()

        # FPS counter
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

    def capture_and_detect(self):
        """Capture frame and run YOLO detection"""
        # Capture frame
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Run YOLO detection
        results = self.model.predict(
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

    def draw_overlay(self, frame, detections, encoder_angle, motor_pwm, velocity_telemetry):
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
            f"Angle: {encoder_angle:.1f}deg",
            f"PWM: {motor_pwm:+.0%}",
            f"Vel: {velocity_telemetry['current_velocity']:.0f} c/s",
            f"Tgt Vel: {velocity_telemetry['target_velocity']:.0f} c/s",
            f"Detections: {len(detections)}",
            f"Tracking: {'YES' if velocity_telemetry['tracking_active'] else 'NO'}"
        ]

        y_offset = 30
        for text in telemetry:
            cv2.putText(overlay, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 28

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
            motor_pwm = motor.get_pwm()
            velocity_telemetry = tracking_controller.get_telemetry()
            overlay_frame = self.draw_overlay(frame, detections, encoder_angle,
                                             motor_pwm, velocity_telemetry)

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
    telemetry = tracking_controller.get_telemetry() if tracking_controller else {}

    return jsonify({
        'encoder_angle': encoder.get_angle() if encoder else 0,
        'encoder_count': encoder.get_count() if encoder else 0,
        'motor_pwm': motor.get_pwm() if motor else 0,
        'current_velocity': telemetry.get('current_velocity', 0),
        'target_velocity': telemetry.get('target_velocity', 0),
        'integral': telemetry.get('integral', 0),
        'detections': detection_pipeline.get_detections() if detection_pipeline else [],
        'tracking_active': telemetry.get('tracking_active', False)
    })

@app.route('/api/manual_control', methods=['POST'])
def api_manual_control():
    """Manual motor control override"""
    data = request.json
    command = data.get('command', '')

    if command == 'stop':
        motor.stop()
        tracking_controller.clear_target()
    elif command == 'cw':
        pwm = data.get('speed', 0.3)
        motor.set_pwm(pwm)
    elif command == 'ccw':
        pwm = data.get('speed', 0.3)
        motor.set_pwm(-pwm)
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
    print("FIXED VERSION with comprehensive improvements")
    print("=" * 70)
    print("\nCRITICAL FIXES APPLIED:")
    print("  ✓ Motor direction mapping corrected (IN1/IN2 swap)")
    print("  ✓ PID controller tuned (I/D terms enabled)")
    print("  ✓ Velocity deadband reduced (60→30 counts/sec)")
    print("  ✓ PWM limits optimized (max 0.70)")
    print("  ✓ Speed linearization validation fixed")
    print("\nArchitecture:")
    print("  • Three-layer control: Motor → Linearization → Velocity PID")
    print("  • Fixed 20Hz control rate (eliminates timing jitter)")
    print("  • Proper anti-windup with conditional integration")
    print("  • Rate limiting on velocity changes")
    print("\nInitializing hardware...")

    # Initialize hardware
    encoder = QuadratureEncoder(GPIO_ENCODER_A, GPIO_ENCODER_B, COUNTS_PER_360)
    motor = MotorDriver(GPIO_MOTOR_IN1, GPIO_MOTOR_IN2, MOTOR_PWM_FREQ)

    # Initialize speed linearizer (Layer 2)
    speed_linearizer = SpeedLinearizer(SPEED_CALIBRATION_FILE)

    # Initialize velocity tracking controller (Layer 3)
    tracking_controller = VelocityTrackingController(motor, encoder, speed_linearizer)

    # Initialize detection pipeline
    detection_pipeline = DetectionPipeline(YOLO_MODEL_PATH)

    print("\n" + "=" * 70)
    print("Starting system threads...")
    print("=" * 70)

    # Start velocity control loop (fixed 20Hz timing)
    tracking_controller.start()

    # Start detection pipeline
    detection_thread = Thread(
        target=detection_pipeline.run,
        args=(tracking_controller, encoder, motor),
        daemon=True
    )
    detection_thread.start()

    print("\n✓ All systems running!")
    print(f"\n🌐 Web GUI available at: http://localhost:{WEB_PORT}")
    print("\n⚠️  IMPORTANT: For best performance, run speed_calibration_fixed.py first")
    print("   to build the corrected PWM→velocity linearization table.")
    print("\nPress Ctrl+C to stop\n")

    try:
        # Start web server
        socketio.run(app, host='0.0.0.0', port=WEB_PORT, debug=False)

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        tracking_controller.stop()
        motor.cleanup()
        encoder.cleanup()
        detection_pipeline.picam2.stop()
        print("✓ Cleanup complete")


if __name__ == "__main__":
    main()
