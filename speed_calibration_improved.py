#!/usr/bin/env python3
"""
PTDTS Speed Calibration Script (IMPROVED)
Builds a valid PWM → velocity lookup table with better testing methodology

Improvements:
- Tests smaller PWM increments for better resolution
- Includes minimum PWM detection
- Validates measurements for consistency
- Detects and reports mechanical issues
"""

import time
import json
import numpy as np
from gpiozero import DigitalInputDevice, PWMOutputDevice
from threading import Lock
import sys

# Calibration constant from motor_calibration.py
COUNTS_PER_360 = -8556  # CW = negative

# GPIO Pin assignments
GPIO_ENCODER_A = 17
GPIO_ENCODER_B = 4
GPIO_MOTOR_IN1 = 27
GPIO_MOTOR_IN2 = 22

# Calibration parameters
MOTOR_PWM_FREQ = 1000  # Hz
MEASUREMENT_DURATION = 2.0  # seconds per test point (reduced for faster calibration)
SETTLING_TIME = 1.0  # seconds to let motor reach steady state
VALIDATION_RUNS = 2  # Number of runs per PWM level for validation

# PWM test points - finer resolution at low speeds
PWM_TEST_POINTS = [
    0.10, 0.12, 0.15, 0.18, 0.20, 0.22, 0.25, 0.28,
    0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60
]


class QuadratureEncoder:
    """Thread-safe interrupt-driven quadrature encoder"""
    
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.lock = Lock()
        
        # Initialize GPIO inputs with pull-ups
        self.encoder_a = DigitalInputDevice(pin_a, pull_up=True)
        self.encoder_b = DigitalInputDevice(pin_b, pull_up=True)
        
        # Attach interrupt handlers
        self.encoder_a.when_activated = self._on_a_rising
        self.encoder_a.when_deactivated = self._on_a_falling
        self.encoder_b.when_activated = self._on_b_rising
        self.encoder_b.when_deactivated = self._on_b_falling
        
        print(f"✓ Encoder initialized on GPIO {pin_a} (A) and GPIO {pin_b} (B)")
    
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
    
    def reset_count(self):
        with self.lock:
            self.count = 0
    
    def cleanup(self):
        self.encoder_a.close()
        self.encoder_b.close()


class MotorDriver:
    """DRV8874 motor driver controller"""
    
    def __init__(self, pin_in1, pin_in2, pwm_frequency=1000):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2
        
        # Initialize PWM outputs
        self.in1 = PWMOutputDevice(pin_in1, frequency=pwm_frequency)
        self.in2 = PWMOutputDevice(pin_in2, frequency=pwm_frequency)
        
        self.stop()
        print(f"✓ Motor driver initialized on GPIO {pin_in1} (IN1) and GPIO {pin_in2} (IN2)")
        print(f"  PWM frequency: {pwm_frequency} Hz")
    
    def set_pwm(self, pwm):
        """
        Set motor PWM duty cycle
        
        Args:
            pwm: -1.0 to 1.0
                 Positive = CW (should produce negative counts)
                 Negative = CCW (should produce positive counts)
        """
        pwm = max(-1.0, min(1.0, pwm))
        
        if pwm > 0:
            # Positive PWM = Clockwise
            self.in1.value = 0
            self.in2.value = abs(pwm)
        elif pwm < 0:
            # Negative PWM = Counter-clockwise
            self.in1.value = abs(pwm)
            self.in2.value = 0
        else:
            # Stop
            self.in1.value = 0
            self.in2.value = 0
    
    def stop(self):
        """Stop motor (coast mode)"""
        self.in1.value = 0
        self.in2.value = 0
    
    def cleanup(self):
        """Stop motor and clean up GPIO"""
        self.stop()
        self.in1.close()
        self.in2.close()


def find_minimum_pwm(encoder, motor, direction=1):
    """
    Find the minimum PWM needed to start the motor moving
    
    Returns:
        Minimum PWM value that produces movement
    """
    print(f"\nFinding minimum PWM for {'CW' if direction > 0 else 'CCW'}...")
    
    min_pwm = 0.0
    for test_pwm in np.arange(0.05, 0.30, 0.01):
        motor.set_pwm(direction * test_pwm)
        time.sleep(0.5)
        
        # Check if motor is moving
        encoder.reset_count()
        time.sleep(0.5)
        counts = abs(encoder.get_count())
        
        if counts > 10:  # Threshold for movement detection
            min_pwm = test_pwm
            print(f"  Minimum PWM found: {min_pwm:.2f}")
            break
    
    motor.stop()
    time.sleep(0.5)
    
    if min_pwm == 0.0:
        print("  ⚠ No movement detected up to PWM 0.30")
        min_pwm = 0.15  # Default fallback
    
    return min_pwm


def measure_velocity_with_validation(encoder, motor, pwm_value, duration, settling_time, runs=2):
    """
    Measure velocity with multiple runs for validation
    
    Returns:
        Average velocity and standard deviation
    """
    velocities = []
    
    for run in range(runs):
        # Set motor PWM
        motor.set_pwm(pwm_value)
        
        # Wait for settling
        time.sleep(settling_time)
        
        # Measure
        encoder.reset_count()
        start_time = time.time()
        time.sleep(duration)
        end_count = encoder.get_count()
        end_time = time.time()
        
        # Calculate velocity
        velocity = end_count / (end_time - start_time)
        velocities.append(velocity)
        
        # Stop between runs
        motor.stop()
        time.sleep(0.5)
    
    avg_velocity = np.mean(velocities)
    std_velocity = np.std(velocities)
    
    return avg_velocity, std_velocity


def validate_calibration_data(lookup_table):
    """
    Validate the calibration data for issues
    
    Returns:
        (is_valid, issues_found)
    """
    issues = []
    
    # Check for zero velocities with non-zero PWM
    zero_vel_count = sum(1 for v, p in lookup_table if v == 0 and p != 0)
    if zero_vel_count > 0:
        issues.append(f"{zero_vel_count} points with zero velocity at non-zero PWM")
    
    # Check for direction inversions
    for v, p in lookup_table:
        if p != 0 and v != 0:
            # Check if signs match (considering negative counts for CW)
            if p > 0 and v > 0:  # Positive PWM should give negative velocity
                issues.append(f"Direction mismatch at PWM {p:.2f}")
            elif p < 0 and v < 0:  # Negative PWM should give positive velocity
                issues.append(f"Direction mismatch at PWM {p:.2f}")
    
    # Check for non-monotonic behavior (velocity should increase with PWM magnitude)
    cw_points = [(abs(v), p) for v, p in lookup_table if p > 0]
    ccw_points = [(abs(v), abs(p)) for v, p in lookup_table if p < 0]
    
    for points, direction in [(cw_points, "CW"), (ccw_points, "CCW")]:
        if len(points) > 1:
            points.sort(key=lambda x: x[1])  # Sort by PWM
            for i in range(1, len(points)):
                if points[i][0] < points[i-1][0] * 0.9:  # Allow 10% tolerance
                    issues.append(f"Non-monotonic velocity in {direction} direction")
                    break
    
    is_valid = len(issues) == 0
    return is_valid, issues


def save_calibration(lookup_table, min_pwm_cw, min_pwm_ccw, filename="speed_linearization_table.json"):
    """Save validated calibration data"""
    
    # Validate before saving
    is_valid, issues = validate_calibration_data(lookup_table)
    
    if not is_valid:
        print("\n⚠ WARNING: Calibration data has issues:")
        for issue in issues:
            print(f"  - {issue}")
        response = input("\nSave anyway? (y/n): ")
        if response.lower() != 'y':
            print("Calibration not saved")
            return None
    
    calibration_data = {
        "lookup_table": [
            {"velocity": v, "pwm": p} for v, p in lookup_table
        ],
        "minimum_pwm_cw": min_pwm_cw,
        "minimum_pwm_ccw": min_pwm_ccw,
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "motor": "30:1 Pololu 37D with 64 CPR encoder",
        "external_gear_ratio": "4.5:1 (24T to 108T)",
        "measurement_duration": MEASUREMENT_DURATION,
        "settling_time": SETTLING_TIME,
        "counts_per_360": COUNTS_PER_360,
        "validation_status": "PASSED" if is_valid else "FAILED"
    }
    
    with open(filename, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    print(f"\n✓ Calibration saved to {filename}")
    return calibration_data


def main():
    print("=" * 70)
    print("PTDTS SPEED CALIBRATION (IMPROVED)")
    print("=" * 70)
    print("\nThis script builds a validated PWM → velocity lookup table")
    print("for accurate motor control and overshoot elimination.")
    print("\n⚠ WARNING: Motor will run automatically!")
    print("   Ensure the system can rotate freely.")
    print("\nPress Ctrl+C at any time to abort")
    print("=" * 70)
    
    # Initialize hardware
    encoder = QuadratureEncoder(pin_a=GPIO_ENCODER_A, pin_b=GPIO_ENCODER_B)
    motor = MotorDriver(pin_in1=GPIO_MOTOR_IN1, pin_in2=GPIO_MOTOR_IN2, 
                       pwm_frequency=MOTOR_PWM_FREQ)
    
    lookup_table = []
    
    try:
        input("\nPress ENTER to start calibration...")
        
        # Find minimum PWM values
        min_pwm_cw = find_minimum_pwm(encoder, motor, direction=1)
        min_pwm_ccw = find_minimum_pwm(encoder, motor, direction=-1)
        
        print(f"\nMinimum PWM values:")
        print(f"  CW:  {min_pwm_cw:.2f}")
        print(f"  CCW: {min_pwm_ccw:.2f}")
        
        # Test zero point
        print(f"\nTest 0: PWM = 0.00 (stopped)")
        lookup_table.append((0.0, 0.0))
        
        # Test clockwise (positive PWM should give negative velocity)
        print("\n--- CLOCKWISE DIRECTION (Positive PWM) ---")
        test_points_cw = [p for p in PWM_TEST_POINTS if p >= min_pwm_cw]
        
        for i, pwm in enumerate(test_points_cw, 1):
            print(f"\nTest {i}/{len(test_points_cw)}: PWM = {pwm:+.2f}")
            
            avg_vel, std_vel = measure_velocity_with_validation(
                encoder, motor, pwm, MEASUREMENT_DURATION, SETTLING_TIME, VALIDATION_RUNS
            )
            
            print(f"  Velocity: {avg_vel:.1f} ± {std_vel:.1f} counts/sec")
            
            # Check for correct direction
            if avg_vel > 100 and pwm > 0:
                print(f"  ⚠ WARNING: Wrong direction! Expected negative velocity")
            
            lookup_table.append((avg_vel, pwm))
            
            # Check for mechanical issues
            if std_vel > abs(avg_vel) * 0.2:
                print(f"  ⚠ High variance - possible mechanical issue")
            
            time.sleep(0.5)
        
        # Test counter-clockwise (negative PWM should give positive velocity)
        print("\n--- COUNTER-CLOCKWISE DIRECTION (Negative PWM) ---")
        test_points_ccw = [p for p in PWM_TEST_POINTS if p >= min_pwm_ccw]
        
        for i, pwm in enumerate(test_points_ccw, 1):
            print(f"\nTest {i}/{len(test_points_ccw)}: PWM = {-pwm:+.2f}")
            
            avg_vel, std_vel = measure_velocity_with_validation(
                encoder, motor, -pwm, MEASUREMENT_DURATION, SETTLING_TIME, VALIDATION_RUNS
            )
            
            print(f"  Velocity: {avg_vel:.1f} ± {std_vel:.1f} counts/sec")
            
            # Check for correct direction
            if avg_vel < -100 and pwm < 0:
                print(f"  ⚠ WARNING: Wrong direction! Expected positive velocity")
            
            lookup_table.append((avg_vel, -pwm))
            
            if std_vel > abs(avg_vel) * 0.2:
                print(f"  ⚠ High variance - possible mechanical issue")
            
            time.sleep(0.5)
        
        print("\n" + "=" * 70)
        print("CALIBRATION COMPLETE")
        print("=" * 70)
        
        # Sort by velocity (not absolute value!)
        lookup_table.sort(key=lambda x: x[0])
        
        # Display results
        print("\nCalibration Results:")
        print(f"{'Velocity (cnt/s)':>20} | {'PWM':>6} | {'Direction':>10}")
        print("-" * 45)
        for velocity, pwm in lookup_table:
            if pwm == 0:
                direction = "STOP"
            elif pwm > 0:
                direction = "CW" if velocity < 0 else "WRONG!"
            else:
                direction = "CCW" if velocity > 0 else "WRONG!"
            
            print(f"{velocity:>20.1f} | {pwm:>6.2f} | {direction:>10}")
        
        # Validate and save
        save_calibration(lookup_table, min_pwm_cw, min_pwm_ccw)
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted")
        motor.stop()
        
    except Exception as e:
        print(f"\n\n⚠ Error: {e}")
        motor.stop()
    
    finally:
        motor.cleanup()
        encoder.cleanup()
        print("\n✓ Cleanup complete")


if __name__ == "__main__":
    main()
