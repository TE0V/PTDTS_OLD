#!/bin/bash
# PTDTS Setup and Installation Script
# Run this script on your Raspberry Pi 5 to set up the system

set -e  # Exit on error

echo "=========================================="
echo "PTDTS Setup Script"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "⚠️  Warning: This doesn't appear to be a Raspberry Pi"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Step 1: Updating system packages..."
sudo apt update

echo ""
echo "Step 2: Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-flask \
    libcamera-apps \
    python3-picamera2

echo ""
echo "Step 3: Installing Python packages..."
pip3 install --break-system-packages \
    gpiozero \
    lgpio \
    ultralytics \
    flask \
    flask-cors \
    flask-socketio \
    python-socketio

echo ""
echo "Step 4: Downloading YOLO model..."
if [ ! -d "yolo11n_ncnn_model" ]; then
    echo "Downloading YOLO11n and converting to NCNN format..."
    yolo export model=yolo11n.pt format=ncnn
    echo "✓ YOLO model downloaded"
else
    echo "✓ YOLO model already exists"
fi

echo ""
echo "Step 5: Setting up directory structure..."
mkdir -p logs
mkdir -p recordings

echo ""
echo "Step 6: Testing camera..."
if libcamera-hello --list-cameras > /dev/null 2>&1; then
    echo "✓ Camera detected"
else
    echo "⚠️  Warning: Camera not detected. Please check camera connection."
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Run calibration: python3 motor_calibration.py"
echo "2. Test motor control: python3 motor_control_test.py"
echo "3. Start tracking: python3 ptdts_main.py"
echo ""
echo "Access web GUI at: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
