#!/usr/bin/env python3
"""
PTDTS - Pan Tilt Drone Detection and Tracking System (FRICTION FIX)
Main tracking engine with static friction compensation

ADDITIONAL FIXES FOR HIGH-FRICTION MOTOR:
1. Static friction breakaway boost (increases PWM when starting from rest)
2. Minimum PWM enforcement in speed linearizer
3. Velocity boost for low-speed commands
4. Adjusted PID gains for sticky motor
5. Reduced deadbands for smoother motion

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
MOTOR_MIN_SPEED = 0.35  # Minimum PWM to overcome static friction (INCREASED)
MOTOR_MAX_SPEED = 0.70  # Maximum for safety

# Control loop timing
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
# SPEED LINEARIZER WITH FRICTION COMPENSATION (LAYER 2)
# ============================================================================

class SpeedLinearizer:
    """
    Speed linearization lookup table with friction compensation

    CRITICAL ADDITION: Enforces minimum PWM to overcome static friction
    """

    def __init__(self, calibration_file=None, min_pwm=0.35):
        self.lookup_table = []
        self.calibrated = False
        self.min_pwm = min_pwm  # Minimum PWM to overcome static friction

        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
        else:
            print("⚠ No speed linearization calibration found")
            print("  Using default linear mapping with friction compensation")
            print(f"  Run speed_calibration_fixed.py to create {SPEED_CALIBRATION_FILE}")
            self._create_default_table()

    def _create_default_table(self):
        """Create a default linear lookup table with minimum PWM"""
        max_velocity = 500  # counts/sec estimate at max PWM

        # CW direction (positive PWM → negative velocity)
        for pwm in np.linspace(self.min_pwm, 1.0, 15):
            velocity = -pwm * max_velocity
            self.lookup_table.append((velocity, pwm))

        # Zero point
        self.lookup_table.append((0.0, 0.0))

        # CCW direction (negative PWM → positive velocity)
        for pwm in np.linspace(self.min_pwm, 1.0, 15):
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

            # Extract minimum PWM from calibration data
            if 'minimum_pwm_cw' in data and 'minimum_pwm_ccw' in data:
                self.min_pwm = max(data['minimum_pwm_cw'], data['minimum_pwm_ccw'])
                print(f"  Calibrated minimum PWM: {self.min_pwm:.2f}")

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
        Convert desired velocity to PWM with friction compensation

        CRITICAL: Enforces minimum PWM to overcome static friction
        Small velocities get boosted to minimum PWM

        Args:
            target_velocity: Desired velocity in counts/sec (signed)

        Returns:
            PWM duty cycle (-1.0 to 1.0)
        """
        if len(self.lookup_table) == 0:
            # Fallback linear mapping
            pwm = target_velocity / 500.0
            if abs(pwm) > 0.01:
                # Enforce minimum PWM
                sign = 1 if pwm > 0 else -1
                pwm = sign * max(abs(pwm), self.min_pwm)
            return np.clip(pwm, -1.0, 1.0)

        # Handle zero velocity
        if abs(target_velocity) < 1.0:
            return 0.0

        # Find bracketing points and interpolate
        if target_velocity <= self.lookup_table[0][0]:
            pwm = self.lookup_table[0][1]
        elif target_velocity >= self.lookup_table[-1][0]:
            pwm = self.lookup_table[-1][1]
        else:
            # Interpolate between points
            pwm = 0.0
            for i in range(len(self.lookup_table) - 1):
                v1, pwm1 = self.lookup_table[i]
                v2, pwm2 = self.lookup_table[i + 1]

                if v1 <= target_velocity <= v2:
                    t = (target_velocity - v1) / (v2 - v1) if v2 != v1 else 0
                    pwm = pwm1 + t * (pwm2 - pwm1)
                    break

        # CRITICAL: Enforce minimum PWM for friction
        if abs(pwm) > 0.01:  # If not essentially zero
            sign = 1 if pwm > 0 else -1
            pwm = sign * max(abs(pwm), self.min_pwm)

        return pwm


# ============================================================================
# MOTOR DRIVER WITH FRICTION COMPENSATION (LAYER 1)
# ============================================================================

class MotorDriver:
    """
    DRV8874 motor driver with static friction compensation

    CRITICAL ADDITION: Tracks motor state and applies breakaway boost
    """

    def __init__(self, pin_in1, pin_in2, pwm_freq=1000, min_pwm=0.35):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2
        self.min_pwm = min_pwm

        # Initialize PWM outputs
        self.in1 = PWMOutputDevice(pin_in1, frequency=pwm_freq)
        self.in2 = PWMOutputDevice(pin_in2, frequency=pwm_freq)

        self.current_pwm = 0.0
        self.last_pwm = 0.0
        self.is_moving = False
        self.lock = Lock()

        self.stop()
        print(f"✓ Motor driver initialized (GPIO {pin_in1}, {pin_in2})")
        print(f"  FRICTION COMPENSATION: min_pwm={min_pwm:.2f}")

    def set_pwm(self, pwm):
        """
        Set motor PWM with friction compensation

        CRITICAL: Applies breakaway boost when starting from rest
        """
        with self.lock:
            pwm = max(-1.0, min(1.0, pwm))

            # Track if motor was stopped
            was_stopped = not self.is_moving

            # Determine if motor will be moving
            self.is_moving = abs(pwm) > 0.01

            # FRICTION COMPENSATION: Apply breakaway boost
            if was_stopped and self.is_moving:
                # Motor starting from rest - boost PWM to overcome static friction
                sign = 1 if pwm > 0 else -1
                pwm = sign * max(abs(pwm), self.min_pwm)
            elif self.is_moving and abs(pwm) > 0.01:
                # Motor already moving - ensure minimum PWM for kinetic friction
                sign = 1 if pwm > 0 else -1
                # Use slightly lower threshold for running (kinetic < static friction)
                min_running_pwm = self.min_pwm * 0.85
                pwm = sign * max(abs(pwm), min_running_pwm)

            self.current_pwm = pwm
            self.last_pwm = pwm

            if pwm > 0:
                # Clockwise
                self.in1.value = 0
                self.in2.value = abs(pwm)
            elif pwm < 0:
                # Counter-clockwise
                self.in1.value = abs(pwm)
                self.in2.value = 0
            else:
                # Stop
                self.in1.value = 0
                self.in2.value = 0
                self.is_moving = False

    def stop(self):
        with self.lock:
            self.current_pwm = 0.0
            self.in1.value = 0
            self.in2.value = 0
            self.is_moving = False

    def get_pwm(self):
        with self.lock:
            return self.current_pwm

    def cleanup(self):
        self.stop()
        self.in1.close()
        self.in2.close()


# ============================================================================
# VELOCITY TRACKING CONTROLLER WITH FRICTION TUNING (LAYER 3)
# ============================================================================

class VelocityTrackingController:
    """
    Velocity-based tracking controller tuned for high-friction motor
    """

    def __init__(self, motor, encoder, speed_linearizer):
        self.motor = motor
        self.encoder = encoder
        self.speed_linearizer = speed_linearizer

        # Tracking state
        self.target_x = None
        self.frame_center_x = CAMERA_WIDTH / 2
        self.tracking_active = False

        # Control parameters - TUNED FOR HIGH FRICTION
        self.deadzone_pixels = 15  # Wider deadzone to prevent constant micro-adjustments
        self.max_velocity = 1000   # Reduced from 1200 (motor struggles at high speed)
        self.max_acceleration = 500  # Gentler acceleration
        self.velocity_deadband = 20  # Very tight deadband (was 30)

        # PID gains - AGGRESSIVE FOR HIGH FRICTION
        self.kp_velocity = 2.0   # MUCH higher (was 1.2) - need strong response
        self.ki_velocity = 0.3   # Higher integral (was 0.15) - overcome friction
        self.kd_velocity = 0.02  # Reduced derivative (was 0.05) - less damping needed

        # Velocity measurement state
        self.last_position = encoder.get_count()
        self.current_velocity = 0.0
        self.target_velocity = 0.0

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
        print(f"  PID gains (HIGH FRICTION): Kp={self.kp_velocity}, Ki={self.ki_velocity}, Kd={self.kd_velocity}")
        print(f"  Velocity deadband: {self.velocity_deadband} counts/sec")

    def update_target(self, target_center_x):
        """Update target position"""
        with self.lock:
            self.target_x = target_center_x
            self.tracking_active = True

    def clear_target(self):
        """Clear target"""
        with self.lock:
            self.target_x = None
            self.tracking_active = False
            self.integral = 0.0

    def _compute_velocity_command(self):
        """Compute velocity control command"""
        with self.lock:
            if not self.tracking_active or self.target_x is None:
                self.integral = 0.0
                self.last_velocity_error = 0.0
                return 0.0

            # Position error to target velocity
            pixel_error = self.frame_center_x - self.target_x

            # Apply deadzone
            if abs(pixel_error) < self.deadzone_pixels:
                self.integral = 0.0
                self.last_velocity_error = 0.0
                return 0.0

            # Adaptive gain based on error magnitude
            if abs(pixel_error) > 150:
                kp_position = 0.8  # Fast for large errors
            elif abs(pixel_error) > 50:
                kp_position = 0.5  # Medium for moderate errors
            else:
                kp_position = 0.3  # Gentle for small errors

            target_velocity = kp_position * pixel_error

            # Clamp to maximum velocity
            target_velocity = max(-self.max_velocity,
                                 min(self.max_velocity, target_velocity))

            return target_velocity

    def _control_loop_update(self):
        """Fixed-rate control loop update"""
        # Measure current velocity
        current_position = self.encoder.get_count()
        position_delta = current_position - self.last_position
        self.current_velocity = position_delta / self.control_period
        self.last_position = current_position

        # Compute target velocity from position error
        target_velocity = self._compute_velocity_command()

        # Apply rate limiting
        velocity_change = target_velocity - self.target_velocity
        max_change = self.max_acceleration * self.control_period

        if abs(velocity_change) > max_change:
            velocity_change = max_change if velocity_change > 0 else -max_change

        self.target_velocity += velocity_change

        # Velocity PID control
        velocity_error = self.target_velocity - self.current_velocity

        # Apply velocity deadband
        if abs(velocity_error) < self.velocity_deadband:
            velocity_error = 0.0

        # Proportional term
        p_term = self.kp_velocity * velocity_error

        # Integral term with anti-windup
        control_velocity_estimate = p_term + self.ki_velocity * self.integral
        output_saturated = abs(control_velocity_estimate) >= self.max_velocity * 0.95

        if not output_saturated:
            self.integral += velocity_error * self.control_period
            max_integral = 300  # Increased for high friction
            self.integral = max(-max_integral, min(max_integral, self.integral))

        i_term = self.ki_velocity * self.integral

        # Derivative term
        velocity_error_rate = (velocity_error - self.last_velocity_error) / self.control_period
        d_term = self.kd_velocity * velocity_error_rate
        self.last_velocity_error = velocity_error

        # Combined control output
        control_velocity = p_term + i_term + d_term

        # Clamp to max velocity
        control_velocity = max(-self.max_velocity,
                              min(self.max_velocity, control_velocity))

        # Convert velocity to PWM (with friction compensation in linearizer)
        pwm_command = self.speed_linearizer.velocity_to_pwm(control_velocity)

        # Limit PWM
        max_pwm = MOTOR_MAX_SPEED
        pwm_command = max(-max_pwm, min(max_pwm, pwm_command))

        # Send to motor (with breakaway boost in motor driver)
        self.motor.set_pwm(pwm_command)

        # Debug logging
        if self.debug_mode:
            self.debug_counter += 1
            if self.debug_counter % 10 == 0:
                pixel_err = self.frame_center_x - self.target_x if self.target_x else 0
                print(f"[FRICTION] px={pixel_err:6.1f} | "
                      f"tgt_v={self.target_velocity:6.0f} | cur_v={self.current_velocity:6.0f} | "
                      f"err={velocity_error:6.0f} | P={p_term:+7.0f} I={i_term:+7.0f} D={d_term:+6.0f} | "
                      f"pwm={pwm_command:+.3f}")

        # Schedule next update
        if self.running:
            self.next_update_time += self.control_period
            delay = self.next_update_time - time.time()

            if delay < 0:
                delay = 0
                self.next_update_time = time.time()

            self.timer = Timer(delay, self._control_loop_update)
            self.timer.daemon = True
            self.timer.start()

    def start(self):
        """Start the control loop"""
        self.running = True
        self.next_update_time = time.time() + self.control_period
        self.last_position = self.encoder.get_count()
        print("✓ Velocity control loop started (20 Hz, friction-compensated)")

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
        """Get current control state"""
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
        time.sleep(2)
        print("✓ Camera initialized")

        # Initialize YOLO
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
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.predict(
            frame,
            conf=YOLO_CONF_THRESHOLD,
            iou=YOLO_IOU_THRESHOLD,
            classes=YOLO_CLASSES,
            verbose=False
        )

        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center': [int(center_x), int(center_y)],
                    'confidence': conf,
                    'class': cls
                })

        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = current_time

        return frame, detections

    def draw_overlay(self, frame, detections, encoder_angle, motor_pwm, velocity_telemetry):
        """Draw overlay"""
        overlay = frame.copy()
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        cv2.drawMarker(overlay, (center_x, center_y), (0, 255, 0),
                      cv2.MARKER_CROSS, 30, 2)

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cx, cy = det['center']
            conf = det['confidence']

            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(overlay, (center_x, center_y), (cx, cy), (255, 0, 0), 2)

            label = f"Drone {conf:.2f}"
            cv2.putText(overlay, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        telemetry = [
            f"FPS: {self.fps}",
            f"Angle: {encoder_angle:.1f}deg",
            f"PWM: {motor_pwm:+.0%}",
            f"Vel: {velocity_telemetry['current_velocity']:.0f} c/s",
            f"Tgt: {velocity_telemetry['target_velocity']:.0f} c/s",
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
        """Get current frame"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            return None

    def get_detections(self):
        """Get current detections"""
        with self.detection_lock:
            return self.detections.copy()

    def run(self, tracking_controller, encoder, motor):
        """Main detection loop"""
        print("✓ Detection pipeline started")

        while True:
            frame, detections = self.capture_and_detect()

            if len(detections) > 0:
                detections.sort(key=lambda d: d['confidence'], reverse=True)
                best_detection = detections[0]
                tracking_controller.update_target(best_detection['center'][0])
            else:
                tracking_controller.clear_target()

            encoder_angle = encoder.get_angle()
            motor_pwm = motor.get_pwm()
            velocity_telemetry = tracking_controller.get_telemetry()
            overlay_frame = self.draw_overlay(frame, detections, encoder_angle,
                                             motor_pwm, velocity_telemetry)

            with self.frame_lock:
                self.current_frame = overlay_frame

            with self.detection_lock:
                self.detections = detections


# ============================================================================
# WEB SERVER
# ============================================================================

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

detection_pipeline = None
tracking_controller = None
encoder = None
motor = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if detection_pipeline is None:
                time.sleep(0.1)
                continue

            frame = detection_pipeline.get_current_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            _, buffer = cv2.imencode('.jpg', frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
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
    data = request.json
    command = data.get('command', '')

    if command == 'stop':
        motor.stop()
        tracking_controller.clear_target()
    elif command == 'cw':
        pwm = data.get('speed', 0.35)
        motor.set_pwm(pwm)
    elif command == 'ccw':
        pwm = data.get('speed', 0.35)
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
    print("FRICTION COMPENSATION VERSION")
    print("=" * 70)
    print("\nFRICTION FIXES APPLIED:")
    print("  ✓ Static friction breakaway boost")
    print("  ✓ Minimum PWM enforcement (35%)")
    print("  ✓ Aggressive PID tuning (Kp=2.0, Ki=0.3)")
    print("  ✓ Kinetic friction compensation")
    print("  ✓ Wider pixel deadzone to reduce jitter")
    print("\nInitializing hardware...")

    # Initialize hardware with friction compensation
    encoder = QuadratureEncoder(GPIO_ENCODER_A, GPIO_ENCODER_B, COUNTS_PER_360)
    motor = MotorDriver(GPIO_MOTOR_IN1, GPIO_MOTOR_IN2, MOTOR_PWM_FREQ, MOTOR_MIN_SPEED)

    # Initialize speed linearizer with friction awareness
    speed_linearizer = SpeedLinearizer(SPEED_CALIBRATION_FILE, MOTOR_MIN_SPEED)

    # Initialize tracking controller
    tracking_controller = VelocityTrackingController(motor, encoder, speed_linearizer)

    # Initialize detection pipeline
    detection_pipeline = DetectionPipeline(YOLO_MODEL_PATH)

    print("\n" + "=" * 70)
    print("Starting system...")
    print("=" * 70)

    tracking_controller.start()

    detection_thread = Thread(
        target=detection_pipeline.run,
        args=(tracking_controller, encoder, motor),
        daemon=True
    )
    detection_thread.start()

    print("\n✓ All systems running!")
    print(f"\n🌐 Web GUI: http://localhost:{WEB_PORT}")
    print("\n💡 FRICTION TIPS:")
    print("   - Motor will always use minimum 35% PWM when moving")
    print("   - Breakaway boost applied when starting from rest")
    print("   - Wider deadzone (15px) reduces micro-movements")
    print("   - Aggressive integral term overcomes sticky friction")
    print("\nPress Ctrl+C to stop\n")

    try:
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
