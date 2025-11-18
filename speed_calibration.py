#!/usr/bin/env python3
"""
PTDTS Speed Linearization Calibration Script

Builds the critical PWM → actual velocity lookup table for Layer 2 control.

This script systematically tests the motor at different PWM levels and measures
the actual velocity achieved using the encoder. The resulting lookup table
compensates for the non-linear motor response that causes overshoot.

From pimotors research:
"Directly controlling duty cycle via PID requires different P, I and D values
for various ranges of duty cycle because the speed of the motor does not respond
linearly to changes in duty cycle."

Hardware:
- 30:1 Pololu Gearmotor with 64 CPR encoder
- External gear reduction: 24T → 108T (4.5:1)
- DRV8874 motor driver
- GPIO 17: Encoder A
- GPIO 4: Encoder B
- GPIO 27: Motor IN1
- GPIO 22: Motor IN2
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
MEASUREMENT_DURATION = 3.0  # seconds per test point
SETTLING_TIME = 1.0  # seconds to let motor reach steady state

# PWM test points (will test both positive and negative)
PWM_TEST_POINTS = [
    0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 
    0.45, 0.50, 0.55, 0.60, 0.65, 0.70
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
                 Positive = CW (counts go negative)
                 Negative = CCW (counts go positive)
        """
        pwm = max(-1.0, min(1.0, pwm))
        
        if pwm > 0:
            # Clockwise
            self.in1.value = abs(pwm)
            self.in2.value = 0
        elif pwm < 0:
            # Counter-clockwise
            self.in1.value = 0
            self.in2.value = abs(pwm)
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


def measure_velocity(encoder, motor, pwm_value, duration, settling_time):
    """
    Measure actual motor velocity at a given PWM level
    
    Args:
        encoder: QuadratureEncoder instance
        motor: MotorDriver instance
        pwm_value: PWM duty cycle to test (-1.0 to 1.0)
        duration: Measurement duration in seconds
        settling_time: Time to wait for motor to reach steady state
    
    Returns:
        Average velocity in counts/sec
    """
    # Set motor to test PWM
    motor.set_pwm(pwm_value)
    
    # Wait for settling
    print(f"  Settling for {settling_time}s...", end='', flush=True)
    time.sleep(settling_time)
    print(" done")
    
    # Reset encoder and start measurement
    encoder.reset_count()
    start_count = encoder.get_count()
    start_time = time.time()
    
    print(f"  Measuring for {duration}s...", end='', flush=True)
    
    # Measure over duration
    time.sleep(duration)
    
    # Get final count and time
    end_count = encoder.get_count()
    end_time = time.time()
    
    # Stop motor
    motor.stop()
    
    # Calculate velocity
    count_delta = end_count - start_count
    time_delta = end_time - start_time
    velocity = count_delta / time_delta
    
    print(f" done")
    print(f"  Result: {count_delta} counts in {time_delta:.2f}s = {velocity:.1f} counts/sec")
    
    return velocity


def save_calibration(lookup_table, filename="speed_linearization_table.json"):
    """Save calibration data to JSON file"""
    calibration_data = {
        "lookup_table": [
            {"velocity": v, "pwm": p} for v, p in lookup_table
        ],
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "motor": "30:1 Pololu 37D with 64 CPR encoder",
        "external_gear_ratio": "4.5:1 (24T to 108T)",
        "measurement_duration": MEASUREMENT_DURATION,
        "settling_time": SETTLING_TIME,
        "counts_per_360": COUNTS_PER_360
    }
    
    with open(filename, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    print(f"\n✓ Calibration saved to {filename}")
    return calibration_data


def main():
    print("=" * 70)
    print("PTDTS SPEED LINEARIZATION CALIBRATION")
    print("=" * 70)
    print("\nThis script builds the PWM → velocity lookup table for Layer 2 control.")
    print("\nThe motor will run at various PWM levels while measuring actual velocity.")
    print("This compensates for non-linear motor response and eliminates overshoot.")
    print("\n⚠️  WARNING: Motor will run automatically at various speeds!")
    print("   Make sure the system is mechanically free to rotate.")
    print("\nCalibration parameters:")
    print(f"  • Test points: {len(PWM_TEST_POINTS)} PWM levels (both directions)")
    print(f"  • Settling time: {SETTLING_TIME}s per point")
    print(f"  • Measurement duration: {MEASUREMENT_DURATION}s per point")
    print(f"  • Total time: ~{len(PWM_TEST_POINTS) * 2 * (SETTLING_TIME + MEASUREMENT_DURATION + 1):.0f}s")
    print("\nPress Ctrl+C at any time to abort")
    print("=" * 70)
    
    # Initialize hardware
    encoder = QuadratureEncoder(pin_a=GPIO_ENCODER_A, pin_b=GPIO_ENCODER_B)
    motor = MotorDriver(pin_in1=GPIO_MOTOR_IN1, pin_in2=GPIO_MOTOR_IN2, 
                       pwm_frequency=MOTOR_PWM_FREQ)
    
    lookup_table = []
    
    try:
        input("\nPress ENTER when ready to start calibration...")
        
        print("\n" + "=" * 70)
        print("STARTING CALIBRATION")
        print("=" * 70)
        
        # Test zero point
        print(f"\nTest 0/{len(PWM_TEST_POINTS)*2}: PWM = 0.00 (stopped)")
        lookup_table.append((0.0, 0.0))
        
        # Test clockwise direction (positive PWM, negative velocity)
        print("\n--- CLOCKWISE DIRECTION (Positive PWM) ---")
        for i, pwm in enumerate(PWM_TEST_POINTS, 1):
            print(f"\nTest {i}/{len(PWM_TEST_POINTS)}: PWM = {pwm:+.2f}")
            velocity = measure_velocity(encoder, motor, pwm, 
                                      MEASUREMENT_DURATION, SETTLING_TIME)
            lookup_table.append((velocity, pwm))
            
            # Brief pause between tests
            time.sleep(1.0)
        
        # Test counter-clockwise direction (negative PWM, positive velocity)
        print("\n--- COUNTER-CLOCKWISE DIRECTION (Negative PWM) ---")
        for i, pwm in enumerate(PWM_TEST_POINTS, 1):
            print(f"\nTest {i}/{len(PWM_TEST_POINTS)}: PWM = {-pwm:+.2f}")
            velocity = measure_velocity(encoder, motor, -pwm, 
                                      MEASUREMENT_DURATION, SETTLING_TIME)
            lookup_table.append((velocity, -pwm))
            
            # Brief pause between tests
            time.sleep(1.0)
        
        print("\n" + "=" * 70)
        print("CALIBRATION COMPLETE")
        print("=" * 70)
        
        # Sort lookup table by velocity for proper interpolation
        lookup_table.sort(key=lambda x: x[0])
        
        # Display results
        print("\nCalibration Results:")
        print(f"{'Velocity (cnt/s)':>20} | {'PWM':>6}")
        print("-" * 30)
        for velocity, pwm in lookup_table:
            print(f"{velocity:>20.1f} | {pwm:>6.2f}")
        
        # Analyze linearity
        velocities = [v for v, _ in lookup_table]
        pwms = [p for _, p in lookup_table]
        
        print("\nAnalysis:")
        print(f"  Velocity range: {min(velocities):.0f} to {max(velocities):.0f} counts/sec")
        print(f"  PWM range: {min(pwms):.2f} to {max(pwms):.2f}")
        
        # Check for non-linearity
        if len(lookup_table) > 3:
            # Simple linearity check: compare actual data to linear fit
            from numpy.polynomial import Polynomial
            p = Polynomial.fit(pwms, velocities, 1)
            predicted = [p(pwm) for pwm in pwms]
            errors = [abs(v - pred) for v, pred in zip(velocities, predicted)]
            max_error = max(errors)
            avg_error = sum(errors) / len(errors)
            
            print(f"  Non-linearity (max deviation from linear): {max_error:.0f} cnt/s")
            print(f"  Average deviation: {avg_error:.0f} cnt/s")
            
            if max_error > 50:
                print("  ✓ Significant non-linearity detected - linearization will help!")
            else:
                print("  ℹ Motor response is fairly linear - linearization may have modest benefit")
        
        # Save calibration
        save_calibration(lookup_table)
        
        print("\n✓ Calibration complete! You can now use ptdts_main_improved.py")
        print("  The speed linearization table will automatically load on startup.")
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
        motor.stop()
        
        if len(lookup_table) > 0:
            response = input("\nSave partial calibration data? (y/n): ")
            if response.lower() == 'y':
                lookup_table.sort(key=lambda x: x[0])
                save_calibration(lookup_table)
                print("✓ Partial calibration saved")
    
    except Exception as e:
        print(f"\n\n⚠ Error during calibration: {e}")
        motor.stop()
    
    finally:
        motor.cleanup()
        encoder.cleanup()
        print("\n✓ Cleanup complete")


if __name__ == "__main__":
    main()
