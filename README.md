# PTDTS - Pan Tilt Drone Detection & Tracking System

**Affordable (~$1000) defensive solution for small unmanned aerial system detection**

Real-time drone tracking using acoustic detection, dual cameras, YOLO AI, and autonomous pan control on Raspberry Pi 5.

---

## 🎯 System Overview

This system combines:
- **Acoustic Detection**: 4-mic ReSpeaker array
- **Visual Detection**: YOLO11n AI on dual cameras (HQ + Global Shutter)
- **Autonomous Tracking**: Continuous motor control for real-time drone centering
- **Precision Hardware**: 30:1 gearmotor with encoder + external 4.5:1 gear reduction

---

## 📦 Hardware Requirements

### Core Components
- **Raspberry Pi 5** (16GB RAM recommended)
- **512GB NVMe SSD** (Sabrent or equivalent)
- **LRS-150 24V Power Supply** (150W)

### Motors & Servos
- **Pan Motor**: Pololu 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder
- **Tilt Servo**: Axon MAX MK2 (requires H25T spline programmer)
- **Motor Driver**: Pololu DRV8874 DC Motor Driver

### Cameras
- **Camera 1**: Raspberry Pi HQ Camera (wide lens for detection + tracking)
- **Camera 2**: Raspberry Pi Global Shutter Camera (tracking)

### Audio
- **Microphone Array**: ReSpeaker XMOS XVF3800 (4-mic array)

### Power Regulation
- **5V Rail**: Pololu D36V50F5 (5.5A for Raspberry Pi)
- **6V Rail**: Pololu D36V50F6 (5.5A for servo)
- **12V Rail**: Pololu D36V50F12 (5.5A for motor driver)

### Mechanical
- **External Gear Reduction**: 24T to 108T (4.5:1 ratio)
- **Total System Reduction**: 135:1 (30:1 × 4.5:1)

---

## ⚡ Quick Start

### 1. Calibrate the Motor

First, run the calibration script to measure exact encoder counts per 360° rotation:

```bash
python3 motor_calibration.py
```

Follow the on-screen instructions:
1. Mark starting position with tape
2. Press ENTER to zero encoder
3. Manually rotate system exactly 360° (back to tape mark)
4. Press ENTER to record count

The script will save `pan_motor_calibration.json` with your calibration constant.

**Expected value**: ~-8556 counts/360° (negative = clockwise)

---

### 2. Test Motor Control

Verify motor control and encoder feedback:

```bash
python3 motor_control_test.py
```

**Controls**:
- ↑/↓ arrows: Control motor speed and direction
- SPACE: Stop motor
- B: Brake motor
- R: Reset encoder
- Q: Quit

**What to verify**:
- Motor direction matches encoder (CW = counts go more negative)
- Smooth acceleration and deceleration
- Encoder counts accurately at low-to-medium speeds
- ~30% minimum speed needed to start from rest
- ~20% speed sufficient once moving

---

### 3. Download YOLO Model

Download the YOLO11n model in NCNN format for optimal ARM64 performance:

```bash
# Install ultralytics if not already installed
pip install ultralytics --break-system-packages

# Download and convert model to NCNN format
yolo export model=yolo11n.pt format=ncnn
```

The NCNN model files will be saved as `yolo11n_ncnn_model/`.

**Note**: You can also train a custom drone detection model using datasets from:
- Roboflow Universe (drone datasets)
- VisDrone dataset
- Custom collected footage

---

### 4. Run the Tracking System

Start the complete tracking system:

```bash
python3 ptdts_main.py
```

The system will:
1. Initialize hardware (encoder, motor, camera)
2. Load YOLO model
3. Start tracking control loop
4. Start detection pipeline
5. Launch web interface

**Access the web GUI**: Open browser to `http://<raspberry-pi-ip>:5000`

---

## 🎮 Using the System

### Web Interface

The web GUI provides:

#### Live Video Feed
- Real-time camera stream with YOLO detection overlays
- Bounding boxes around detected drones
- Center crosshair with tracking line
- FPS counter

#### Telemetry Panel
- **Pan Angle**: Current encoder position (0-360°)
- **Motor Speed**: Current motor power (-100% to +100%)
- **Detections**: Number of objects detected
- **Encoder Count**: Raw encoder count
- **Update Rate**: Telemetry refresh rate (Hz)

#### Manual Controls
- **Arrow Keys/Buttons**: Manual pan control
- **Space/Stop Button**: Stop motor
- **Reset Button**: Zero the encoder
- **Tracking Status**: Shows if system is actively tracking

#### Keyboard Shortcuts
- ↑/← : Pan counter-clockwise
- ↓/→ : Pan clockwise
- SPACE : Stop motor
- R : Reset encoder

---

## 🔧 Configuration

Edit `ptdts_config.ini` to tune system parameters:

### Motor Tuning
```ini
MOTOR_MIN_SPEED = 0.30      # Increase if motor doesn't start reliably
MOTOR_TRACKING_SPEED = 0.20 # Base speed for smooth tracking
MOTOR_MAX_SPEED = 0.60      # Decrease for slower maximum speed
MOTOR_DEADZONE = 20         # Increase to reduce jitter (pixels)
TRACKING_KP = 0.003         # Increase for faster response
```

### Camera Settings
```ini
CAMERA_WIDTH = 1280         # Lower for higher FPS
CAMERA_HEIGHT = 720
CAMERA_FPS = 60            # Target FPS
```

### Detection Tuning
```ini
YOLO_CONF_THRESHOLD = 0.25  # Increase to reduce false positives
YOLO_IOU_THRESHOLD = 0.45   # NMS threshold
```

---

## 🧠 How It Works

### Control Philosophy

**From plan.txt: Key design principle**:
> "The DC motor should be built around the action of first panning to an angle given the encoder steps, but after that it should not be bound by angle but instead try to constantly center the camera on object."

The system uses **continuous control**, NOT position-based control:

1. **Detection Phase**: YOLO detects drone in frame
2. **Error Calculation**: Calculate pixel distance from frame center
3. **Continuous Motor Command**: Generate motor speed proportional to error
4. **Tracking**: Motor constantly adjusts to keep target centered
5. **Encoder Telemetry**: Angle used for display/logging, NOT control

### Smart Motor Control

The system implements intelligent speed control:

```python
# Static friction breakaway
if stopped and speed > 0:
    speed = max(speed, 30%)  # Ensure breakaway torque

# Proportional control
speed = kp × pixel_error

# Speed limits
speed = clamp(speed, -60%, +60%)
```

**Deadzone**: If target is within 20 pixels of center, motor stops (prevents jitter)

### Multi-Threading Architecture

```
Main Thread
├── Detection Thread (Camera + YOLO)
│   ├── Capture frame
│   ├── Run YOLO inference
│   ├── Update tracking target
│   └── Generate overlay
│
├── Tracking Control Thread
│   ├── Read target position
│   ├── Calculate motor command
│   └── Update motor speed (100 Hz)
│
└── Web Server Thread
    ├── Stream video feed
    ├── Serve telemetry API
    └── Handle manual controls
```

---

## 📊 Performance Targets

Based on plan.txt requirements:

**Priority #1: Real-time tracking**
- Target: 30-60 FPS detection
- Motor control: 100 Hz update rate
- Latency: <50ms camera-to-motor

**Secondary: Accuracy**
- Deadzone: ±20 pixels (~1.6° at 1280px width)
- Tracking smoothness over precision

---

## 🐛 Troubleshooting

### Motor doesn't move or moves erratically
1. Check that 12V power supply is connected to DRV8874 VM pin
2. Verify PMODE and SLEEP are tied to 3.3V (always HIGH)
3. Check motor connections to OUT1 and OUT2
4. Increase `MOTOR_MIN_SPEED` if motor stalls at low power

### Encoder counts incorrectly
1. Verify encoder VCC is connected to 3.3V (NOT 5V)
2. Check encoder ground connection
3. Run calibration again at slower rotation speed
4. If counts drift, check for electrical noise on encoder lines

### Camera not detected
1. Verify camera ribbon cable is fully inserted
2. Enable camera in `raspi-config`
3. Check that Picamera2 is installed: `pip show picamera2`
4. Try `libcamera-hello` to verify camera hardware

### YOLO model not found
1. Verify model path in `ptdts_config.ini`
2. Check model file exists: `ls yolo11n_ncnn_model/`
3. Ensure NCNN format (not .pt format)

### Web interface not accessible
1. Check that port 5000 is not blocked by firewall
2. Use Raspberry Pi's IP address, not `localhost`
3. Check Flask is running: `ps aux | grep python`

### System crashes or freezes
1. Check power supply can deliver 150W (6.25A at 24V)
2. Monitor temperature: `vcgencmd measure_temp`
3. Reduce camera resolution for lower CPU load
4. Check for memory issues: `free -h`

---

## 🔒 Safety Notes

1. **Power Sequencing**: Always power on 24V supply before Raspberry Pi
2. **Motor Stall Protection**: Never continuously stall motor (causes overheating)
3. **Encoder Voltage**: Use 3.3V only (5V will damage encoder inputs)
4. **Servo Programming**: Axon MAX MK2 requires programmer before use
5. **Decoupling Capacitors**: Essential for stable operation - do not omit

---

## 📁 File Structure

```
ptdts_framework/
├── motor_calibration.py      # Step 1: Encoder calibration
├── motor_control_test.py     # Step 2: Motor control verification
├── ptdts_main.py             # Step 3: Main tracking system
├── ptdts_config.ini          # Configuration parameters
├── templates/
│   └── index.html            # Web GUI interface
├── pan_motor_calibration.json # Generated calibration data
└── README.md                 # This file
```

---

## 🚀 Future Enhancements

- [ ] Tilt axis control (Axon MAX MK2 servo)
- [ ] Acoustic detection integration (ReSpeaker array)
- [ ] Dual-camera switching (HQ → Global Shutter)
- [ ] Kalman filtering for smoother tracking
- [ ] Recording and logging capabilities
- [ ] Multiple drone tracking
- [ ] Custom trained drone detection model

---

## 📝 Technical Specifications

### Motor System
- **Gear Ratio**: 135:1 total (30:1 internal × 4.5:1 external)
- **Encoder Resolution**: 1920 CPR motor shaft × 4.5 = 8640 CPR output shaft
- **Calibrated Resolution**: 8556 counts/360° (measured)
- **Angular Resolution**: 0.042°/count
- **No-Load Speed**: ~2.4 RPM at 12V (with external gears)
- **Max Torque**: ~630 kg⋅cm (with external gears)

### Camera System
- **Resolution**: 1280×720 (configurable)
- **Frame Rate**: 30-60 FPS (hardware dependent)
- **Field of View**: Depends on lens (wide angle recommended)

### Detection System
- **Model**: YOLO11n (nano variant)
- **Format**: NCNN (ARM64 optimized)
- **Inference Time**: ~50-100ms per frame on RPi 5
- **Classes**: Configurable (all COCO classes by default)

---

## 📄 License

This project is for educational and defensive purposes only.

---

## 🙏 Acknowledgments

- **Hardware**: Pololu robotics components
- **AI**: Ultralytics YOLO
- **Camera**: Raspberry Pi camera ecosystem
- **Community**: Thanks to 11 hours of debugging! 🎉

---

**Built with determination and coffee ☕**

*"The most important outcome of this system is its ability to track movement in real time."* - plan.txt
