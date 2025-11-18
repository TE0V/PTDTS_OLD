#!/usr/bin/env python3
"""
PTDTS Pan Motor Calibration Script
Measures exact encoder counts per 360° rotation

Hardware:
- 30:1 Pololu Gearmotor with 64 CPR encoder
- External gear reduction: 24T → 108T (4.5:1)
- Total theoretical: 1920 × 4.5 = 8640 counts/360°
- GPIO 17: Encoder A
- GPIO 4: Encoder B
"""

import time
import json
from gpiozero import DigitalInputDevice
from signal import pause

class QuadratureEncoder:
    """Handles quadrature encoder with interrupt-driven counting"""
    
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        
        # Initialize GPIO inputs with pull-ups
        self.encoder_a = DigitalInputDevice(pin_a, pull_up=True)
        self.encoder_b = DigitalInputDevice(pin_b, pull_up=True)
        
        # Attach interrupt handlers to both edges of both channels
        self.encoder_a.when_activated = self._on_a_rising
        self.encoder_a.when_deactivated = self._on_a_falling
        self.encoder_b.when_activated = self._on_b_rising
        self.encoder_b.when_deactivated = self._on_b_falling
        
        print(f"✓ Encoder initialized on GPIO {pin_a} (A) and GPIO {pin_b} (B)")
        print(f"  Initial state: A={self.encoder_a.value}, B={self.encoder_b.value}")
    
    def _on_a_rising(self):
        """A channel rising edge"""
        if self.encoder_b.value:
            self.count -= 1  # Counter-clockwise
        else:
            self.count += 1  # Clockwise
    
    def _on_a_falling(self):
        """A channel falling edge"""
        if self.encoder_b.value:
            self.count += 1  # Clockwise
        else:
            self.count -= 1  # Counter-clockwise
    
    def _on_b_rising(self):
        """B channel rising edge"""
        if self.encoder_a.value:
            self.count += 1  # Clockwise
        else:
            self.count -= 1  # Counter-clockwise
    
    def _on_b_falling(self):
        """B channel falling edge"""
        if self.encoder_a.value:
            self.count -= 1  # Counter-clockwise
        else:
            self.count += 1  # Clockwise
    
    def get_count(self):
        """Return current encoder count"""
        return self.count
    
    def reset_count(self):
        """Reset encoder count to zero"""
        self.count = 0
        print("✓ Encoder count reset to 0")
    
    def get_angle(self, counts_per_revolution):
        """Calculate angle in degrees"""
        return (self.count / counts_per_revolution) * 360.0
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.encoder_a.close()
        self.encoder_b.close()


def save_calibration(counts_per_360, filename="pan_motor_calibration.json"):
    """Save calibration data to JSON file"""
    calibration_data = {
        "counts_per_360": counts_per_360,
        "degrees_per_count": 360.0 / counts_per_360,
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "motor": "30:1 Pololu 37D with 64 CPR encoder",
        "external_gear_ratio": "4.5:1 (24T to 108T)",
        "total_reduction": "135:1",
        "theoretical_counts": 8640
    }
    
    with open(filename, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    print(f"\n✓ Calibration saved to {filename}")
    return calibration_data


def main():
    print("=" * 60)
    print("PTDTS PAN MOTOR CALIBRATION")
    print("=" * 60)
    print("\nThis script measures the exact encoder counts per 360° rotation.")
    print("\nINSTRUCTIONS:")
    print("1. Mark the current position with tape")
    print("2. Press ENTER to zero the encoder")
    print("3. Manually rotate the system exactly 360° (back to tape mark)")
    print("4. Press ENTER to record the count")
    print("5. Calibration constant will be calculated and saved")
    print("\nPress Ctrl+C at any time to exit")
    print("=" * 60)
    
    # Initialize encoder
    encoder = QuadratureEncoder(pin_a=17, pin_b=4)
    
    try:
        input("\nPress ENTER when ready to start calibration...")
        encoder.reset_count()
        
        print("\n" + "=" * 60)
        print("ROTATE THE SYSTEM 360° (back to tape mark)")
        print("=" * 60)
        print("\nLive encoder count (updates every 0.5s):")
        print("Press ENTER when you complete the rotation\n")
        
        # Display live count while waiting
        start_time = time.time()
        last_count = 0
        
        # Non-blocking input check
        import sys
        import select
        
        while True:
            # Check if Enter was pressed
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                break
            
            current_count = encoder.get_count()
            if current_count != last_count:
                elapsed = time.time() - start_time
                print(f"\rCount: {current_count:6d}  |  Time: {elapsed:.1f}s", end='', flush=True)
                last_count = current_count
            
            time.sleep(0.05)  # 20 Hz update rate
        
        # Record final count
        final_count = encoder.get_count()
        print(f"\n\n{'=' * 60}")
        print("CALIBRATION COMPLETE")
        print("=" * 60)
        print(f"\nMeasured encoder counts for 360°: {final_count}")
        print(f"Theoretical count (8640):          {8640}")
        print(f"Difference:                        {final_count - 8640} counts")
        print(f"Error:                             {abs((final_count - 8640) / 8640 * 100):.2f}%")
        
        if abs(final_count) < 100:
            print("\n⚠ WARNING: Count seems too low. Did you rotate 360°?")
            response = input("Save this calibration anyway? (y/n): ")
            if response.lower() != 'y':
                print("Calibration cancelled.")
                return
        
        # Calculate calibration constant
        degrees_per_count = 360.0 / final_count
        print(f"\nCalibration constant: {degrees_per_count:.6f} degrees/count")
        
        # Save calibration
        save_calibration(final_count)
        
        print("\n✓ Calibration complete! You can now use this value in your tracking code.")
        print(f"  Use: COUNTS_PER_360 = {final_count}")
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
    
    finally:
        encoder.cleanup()
        print("\n✓ Cleanup complete")


if __name__ == "__main__":
    main()
