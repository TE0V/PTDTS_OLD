# PTDTS Quick Start Guide (FIXED Version)

## 🚀 Quick Setup (3 Steps)

### Step 1: Run Speed Calibration (REQUIRED)

```bash
python3 speed_calibration_fixed.py
```

**Expected result:** Creates `speed_linearization_table_fixed.json` with status: `"validation_status": "PASSED"`

⏱️ Takes ~5-10 minutes

---

### Step 2: Run Main Tracking System

**Choose based on your motor friction:**

#### Option A: Normal Motor (Low Friction)
```bash
python3 ptdts_main_FIXED.py
```

#### Option B: High-Friction Motor (RECOMMENDED if tracking is jittery/segmented)
```bash
python3 ptdts_main_FRICTION_FIX.py
```

**Use FRICTION_FIX if you see:**
- ❌ Jittery, segmented tracking
- ❌ Motor struggles to start moving
- ❌ Stops and starts repeatedly
- ❌ PWM output seems too low to move motor

**Expected output:**
```
✓ Encoder initialized (GPIO 17, 4)
✓ Motor driver initialized (GPIO 27, 22)
  FIXED: Corrected direction mapping
✓ Speed linearization loaded: 23 points
✓ Velocity tracking controller initialized
  PID gains: Kp=1.2, Ki=0.15, Kd=0.05
✓ Camera initialized
✓ YOLO model loaded
✓ All systems running!

🌐 Web GUI available at: http://localhost:5000
```

---

### Step 3: Open Web GUI

Open browser to: `http://<raspberry-pi-ip>:5000`

---

## ✅ What Was Fixed?

| Issue | Status |
|-------|--------|
| Motor rotating wrong direction | ✅ FIXED |
| Speed calibration validation failing | ✅ FIXED |
| PID integral/derivative disabled | ✅ FIXED |
| Velocity deadband too large | ✅ FIXED |
| PWM limits too restrictive | ✅ FIXED |

---

## 🔍 Quick Verification

### Test Motor Direction

In web GUI, click manual controls:

- **CW button** → Motor rotates clockwise, encoder counts go **negative** ✓
- **CCW button** → Motor rotates counter-clockwise, encoder counts go **positive** ✓

### Test Tracking

Point camera at object (any detectable item):

1. YOLO draws bounding box ✓
2. Motor automatically centers target ✓
3. Pixel error decreases to ~0 ✓
4. Smooth motion, no oscillation ✓

---

## 📊 Expected Performance

- **Detection**: 30-60 FPS
- **Control loop**: 20 Hz (50ms period)
- **Tracking accuracy**: ±8 pixels
- **Settling time**: 0.5-1.0 seconds
- **Latency**: ~150ms camera-to-motor

---

## 🐛 Troubleshooting

### "No speed linearization calibration found"

**Fix:** Run `python3 speed_calibration_fixed.py` first

### Motor rotates wrong direction (still)

**Possible cause:** Physical wiring reversed

**Fix:** Swap motor wires at motor terminals

### Tracking oscillates/unstable

**Try:**
1. Decrease `kp_velocity` (line 405 in ptdts_main_FIXED.py)
2. Increase `kd_velocity` (line 408 in ptdts_main_FIXED.py)

### Motor doesn't respond to small movements

**Try:**
- Decrease `deadzone_pixels` (line 393)
- Decrease `velocity_deadband` (line 396)

---

## 📁 Files Overview

### Fixed Files (USE THESE)
- ✅ `speed_calibration_fixed.py` - Corrected calibration script
- ✅ `ptdts_main_FIXED.py` - Fixed main tracking system
- ✅ `FIXES_APPLIED.md` - Complete documentation
- ✅ `QUICK_START_FIXED.md` - This guide

### Original Files (PRESERVED)
- 📦 `speed_calibration_improved.py` - Original (has bugs)
- 📦 `ptdts_main_improved-2.py` - Original (has bugs)
- 📦 `speed_linearization_table.json` - Failed calibration

---

## 🎮 Web GUI Controls

### Keyboard Shortcuts
- `←` or `↑` : Pan counter-clockwise
- `→` or `↓` : Pan clockwise
- `SPACE` : Stop motor
- `R` : Reset encoder to zero

### Telemetry Display
- **FPS**: Detection frame rate
- **Angle**: Current pan position (0-360°)
- **PWM**: Motor power (-100% to +100%)
- **Vel**: Current velocity (counts/sec)
- **Tgt Vel**: Target velocity (counts/sec)
- **Detections**: Number of objects detected
- **Tracking**: YES/NO

---

## 📈 Monitoring Health

### Good Debug Output
```
[VEL] px_err= 120.0 | tgt_v=  -72 | cur_v=  -68 | v_err=   -4 | P=  -5 I=  -2 D=  -1 | pwm=-0.35
[VEL] px_err=  45.0 | tgt_v=  -27 | cur_v=  -25 | v_err=   -2 | P=  -2 I=  -1 D=   0 | pwm=-0.22
[VEL] px_err=   2.0 | tgt_v=    0 | cur_v=    0 | v_err=    0 | P=   0 I=   0 D=   0 | pwm= 0.00
```
✅ Pixel error converging to 0
✅ Velocity tracking target
✅ PID terms all active (P, I, D)

### Bad Debug Output
```
[VEL] px_err= 120.0 | tgt_v=  -72 | cur_v=   65 | v_err= -137 | P=-109 I=   0 D=   0 | pwm= 0.60
[VEL] px_err= 150.0 | tgt_v=  -90 | cur_v=   80 | v_err= -170 | P=-136 I=   0 D=   0 | pwm= 0.60
```
❌ Pixel error increasing (motor going wrong way)
❌ Velocity opposite to target
❌ I and D terms not working

→ Re-run calibration or check motor wiring

---

## ⚙️ Configuration Tuning

### For Faster Tracking
```python
# In ptdts_main_FIXED.py
self.kp_velocity = 1.5      # line 405 (increase from 1.2)
self.max_velocity = 1500    # line 395 (increase from 1200)
MOTOR_MAX_SPEED = 0.75      # line 54 (increase from 0.70)
```

### For Smoother Tracking
```python
self.kd_velocity = 0.08     # line 408 (increase from 0.05)
self.max_acceleration = 500 # line 396 (decrease from 600)
```

### For Tighter Accuracy
```python
self.deadzone_pixels = 5    # line 393 (decrease from 8)
self.velocity_deadband = 20 # line 396 (decrease from 30)
```

---

## 🔧 Hardware Checklist

Before running system:

- [ ] 24V power supply connected and ON
- [ ] Raspberry Pi 5 powered via 5V regulator
- [ ] Motor driver receiving 12V at VM pin
- [ ] Encoder connected to 3.3V (NOT 5V!)
- [ ] GPIO pins correct:
  - Encoder A: GPIO 17
  - Encoder B: GPIO 4
  - Motor IN1: GPIO 27
  - Motor IN2: GPIO 22
- [ ] Camera ribbon cable fully inserted
- [ ] System can rotate freely (no obstructions)

---

## 📞 Need Help?

1. Read [FIXES_APPLIED.md](FIXES_APPLIED.md) for detailed explanations
2. Check debug output in console
3. Verify hardware connections
4. Re-run speed calibration
5. Check web GUI telemetry

---

## 🎯 Success Criteria

Your system is working correctly when:

✅ Speed calibration passes validation
✅ Motor rotates in correct direction (CW = negative counts)
✅ Tracking centers objects smoothly
✅ No oscillation or overshoot
✅ Pixel error converges to <10 pixels
✅ All PID terms (P, I, D) are active

---

**Ready to track some drones!** 🚁

*Last updated: 2025-11-17*
