#!/usr/bin/env bash
set -e

echo "[1/4] Installing system packages..."
sudo apt update
sudo apt install -y python3-venv python3-pip v4l-utils

echo "[2/4] Creating virtual environment..."
python3 -m venv .venv

echo "[3/4] Installing Python packages..."
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt

echo "[4/4] Done."
echo ""
echo "Run:"
echo "source /opt/ros/humble/setup.bash"
echo "source .venv/bin/activate"
echo "export ROS_DOMAIN_ID=142"
echo "CAMERA_SOURCE=0 MIRROR_VIEW=0 python hand_tracker.py"
