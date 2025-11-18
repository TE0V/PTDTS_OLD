#!/usr/bin/env python3
"""
PTDTS Pan Motor Control Test Script
Tests precise motor control with encoder feedback

Hardware:
- DRV8874 Motor Driver (PWM mode with PMODE=1)
- GPIO 27: IN1 (motor control)
- GPIO 22: IN2 (motor control)
- GPIO 17: Encoder A
- GPIO 4: Encoder B

Controls:
- UP/DOWN arrows: Control motor speed/direction
- SPACE: Stop motor
- R: Reset encoder to zero
- Q: Quit
"""

import time
import sys
import tty
import termios
from gpiozero import DigitalInputDevice, PWMOutputDevice, DigitalOutputDevice
from threading import Lock

# Calibration constant from calibration script
COUNTS_PER_360 = -8556  # Clockwise = negative


class QuadratureEncoder:
    """Interrupt-driven quadrature encoder with thread-safe counting"""
    
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.lock = Lock()  # Thread-safe counting
        
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
            if self.encoder_b.value:
                self.count -= 1  # CW
            else:
                self.count += 1  # CCW
    
    def _on_a_falling(self):
        with self.lock:
            if self.encoder_b.value:
                self.count += 1  # CCW
            else:
                self.count -= 1  # CW
    
    def _on_b_rising(self):
        with self.lock:
            if self.encoder_a.value:
                self.count += 1  # CCW
            else:
                self.count -= 1  # CW
    
    def _on_b_falling(self):
        with self.lock:
            if self.encoder_a.value:
                self.count -= 1  # CW
            else:
                self.count += 1  # CCW
    
    def get_count(self):
        with self.lock:
            return self.count
    
    def reset_count(self):
        with self.lock:
            self.count = 0
    
    def get_angle(self):
        """Return angle in degrees (0-360 wrapping)"""
        with self.lock:
            angle = (self.count / COUNTS_PER_360) * 360.0
            # Normalize to 0-360 range
            return angle % 360.0
    
    def cleanup(self):
        self.encoder_a.close()
        self.encoder_b.close()


class MotorDriver:
    """
    DRV8874 Motor Driver Controller
    
    PMODE=1 (tied to 3.3V): PWM IN/IN control mode
    
    Control Logic (from DRV8874 datasheet):
    | IN1 | IN2 | Action              | Notes                    |
    |-----|-----|---------------------|--------------------------|
    | 0   | 0   | Coast               | Motor freewheels         |
    | PWM | 0   | Forward/Coast       | Speed = PWM%             |
    | 0   | PWM | Reverse/Coast       | Speed = PWM%             |
    | 1   | 1   | Brake               | Active brake (short)     |
    | PWM | 1   | Reverse/Brake       | Speed = 100%-PWM%        |
    | 1   | PWM | Forward/Brake       | Speed = 100%-PWM%        |
    
    We'll use the simple PWM/0 and 0/PWM modes for forward/reverse.
    """
    
    def __init__(self, pin_in1, pin_in2, pwm_frequency=1000):
        self.pin_in1 = pin_in1
        self.pin_in2 = pin_in2
        
        # Initialize as PWM outputs
        self.in1 = PWMOutputDevice(pin_in1, frequency=pwm_frequency)
        self.in2 = PWMOutputDevice(pin_in2, frequency=pwm_frequency)
        
        self.current_speed = 0.0  # -1.0 to 1.0
        self.stop()
        
        print(f"✓ Motor driver initialized on GPIO {pin_in1} (IN1) and GPIO {pin_in2} (IN2)")
        print(f"  PWM frequency: {pwm_frequency} Hz")
    
    def set_speed(self, speed):
        """
        Set motor speed and direction
        
        Args:
            speed: -1.0 to 1.0
                   Negative = CCW (counts go positive)
                   Positive = CW (counts go negative)
                   0 = stop (coast)
        """
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))
        self.current_speed = speed
        
        if speed > 0:
            # Clockwise (counts go negative)
            # IN1=PWM, IN2=0
            self.in1.value = abs(speed)
            self.in2.value = 0
        elif speed < 0:
            # Counter-clockwise (counts go positive)
            # IN1=0, IN2=PWM
            self.in1.value = 0
            self.in2.value = abs(speed)
        else:
            # Stop (coast)
            self.in1.value = 0
            self.in2.value = 0
    
    def stop(self):
        """Stop motor (coast mode)"""
        self.set_speed(0)
    
    def brake(self):
        """Active brake (both outputs high)"""
        self.in1.value = 1.0
        self.in2.value = 1.0
        self.current_speed = 0.0
    
    def cleanup(self):
        """Stop motor and clean up GPIO"""
        self.stop()
        self.in1.close()
        self.in2.close()


def get_key():
    """Get a single keypress without waiting for Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main():
    print("=" * 70)
    print("PTDTS PAN MOTOR CONTROL TEST")
    print("=" * 70)
    print("\nThis script tests motor control with live encoder feedback")
    print("\nCONTROLS:")
    print("  ↑ (UP)      - Increase speed clockwise (counts go negative)")
    print("  ↓ (DOWN)    - Increase speed counter-clockwise (counts go positive)")
    print("  SPACE       - Stop motor (coast)")
    print("  B           - Brake motor (active brake)")
    print("  R           - Reset encoder to zero")
    print("  Q           - Quit")
    print("\nSpeed increments by 10% per keypress (up to 100%)")
    print("=" * 70)
    
    # Initialize hardware
    encoder = QuadratureEncoder(pin_a=17, pin_b=4)
    motor = MotorDriver(pin_in1=27, pin_in2=22, pwm_frequency=1000)
    
    current_speed = 0.0
    speed_increment = 0.1
    
    try:
        input("\nPress ENTER to start motor control test...")
        print("\nMotor control active! Use arrow keys to control.")
        print("(Press Q to quit)\n")
        
        last_count = encoder.get_count()
        last_display_time = time.time()
        
        while True:
            # Non-blocking key check
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = get_key()
                
                if key == 'q' or key == 'Q':
                    print("\nQuitting...")
                    break
                
                elif key == '\x1b':  # Escape sequence (arrow keys)
                    next_key = get_key()
                    if next_key == '[':
                        arrow = get_key()
                        
                        if arrow == 'A':  # UP arrow - CW (counts go negative)
                            current_speed += speed_increment
                            current_speed = min(1.0, current_speed)
                            motor.set_speed(current_speed)
                            print(f"→ Speed: {current_speed:+.1%} (CW)")
                        
                        elif arrow == 'B':  # DOWN arrow - CCW (counts go positive)
                            current_speed -= speed_increment
                            current_speed = max(-1.0, current_speed)
                            motor.set_speed(current_speed)
                            print(f"← Speed: {current_speed:+.1%} (CCW)")
                
                elif key == ' ':  # Space - stop
                    current_speed = 0.0
                    motor.stop()
                    print("⏸  Motor stopped (coast)")
                
                elif key == 'b' or key == 'B':  # Brake
                    current_speed = 0.0
                    motor.brake()
                    print("⏹  Motor brake engaged")
                
                elif key == 'r' or key == 'R':  # Reset encoder
                    encoder.reset_count()
                    print("↻ Encoder reset to 0")
            
            # Display live feedback every 0.2 seconds
            current_time = time.time()
            if current_time - last_display_time >= 0.2:
                current_count = encoder.get_count()
                angle = encoder.get_angle()
                count_delta = current_count - last_count
                
                # Calculate approximate RPM from count change
                time_delta = current_time - last_display_time
                counts_per_sec = count_delta / time_delta
                rpm = (counts_per_sec / abs(COUNTS_PER_360)) * 60.0
                
                print(f"\rCount: {current_count:6d} | Angle: {angle:6.1f}° | RPM: {rpm:+5.1f} | Speed: {current_speed:+.1%}  ",
                      end='', flush=True)
                
                last_count = current_count
                last_display_time = current_time
            
            time.sleep(0.01)  # 100 Hz loop
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        motor.cleanup()
        encoder.cleanup()
        print("\n✓ Cleanup complete")


if __name__ == "__main__":
    main()
