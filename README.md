# Self-Driving Car using Lane and Object Detection
A compact and cost-effective autonomous car prototype built using Raspberry Pi 4. This system integrates computer vision for lane detection, YOLOv5 for object detection, and ultrasonic sensors for obstacle avoidance — all controlled via a Flask-based web dashboard for live monitoring.


# Table of Contents
About the Project

Features

Hardware Used

Software Stack

System Architecture

Installation

How It Works

Results

Future Scope

Team

# About the Project
This project demonstrates a real-time self-driving car prototype that can:

Detect road lanes using a webcam and OpenCV.

Detect and classify obstacles using YOLOv5n.

Avoid collisions using ultrasonic sensors.

Stream live video to a web browser via Flask and VPN.

Navigate autonomously based on computer vision and distance sensing.

# Features
✅ Lane detection using grayscale + ROI masking

✅ Lightweight YOLOv5n for fast object detection

✅ Collision avoidance with HC-SR04 ultrasonic sensors

✅ DC motor control using L298N motor driver

✅ Flask web dashboard with MJPEG streaming

✅ Threaded video processing to reduce lag

# Hardware Used
Component	Description
Raspberry Pi 4 (4GB)	Central processing unit
USB Camera	Captures live video feed
L298N Motor Driver	Controls 2 DC motors
DC Motors	Drives the chassis
HC-SR04 Sensors (x3)	For front, left, and right obstacle detection
Custom Chassis	6-wheel robot base
Power Supply	5V / 3.3V regulated input

# Software Stack
Python 3

OpenCV – Lane detection

YOLOv5n – Object detection

Flask – Web interface

Threading – For optimizing performance

MJPEG – Efficient video streaming

VPN – Secure remote access

# System Architecture
Camera → OpenCV (Lane Detection)\
       → YOLOv5n (Object Detection)\
Ultrasonic Sensors → Obstacle Distance\
Raspberry Pi 4 → Control Algorithm (Python)\
              → L298N Motor Driver\
              → Flask Web Interface → Browser Dashboard\
# Installation
Copy\
Edit\
\# Clone this repository\
git clone https://github.com/amogh-gadad/self-driving-car-rpi.git\
cd self-driving-car-rpi\

\# Create and activate a virtual environment\
python3 -m venv env\
source env/bin/activate\

\# Install dependencies\
pip install -r requirements.txt\

\# Run the application\
python Final_Code.py

# How It Works
Webcam captures real-time road view.

OpenCV processes the frames for lane boundaries.

YOLOv5n detects obstacles (cars, pedestrians, etc.).

Ultrasonic sensors detect nearby objects (front/left/right).

A Python control algorithm fuses all inputs to navigate safely.

Motor commands are sent to L298N driver via GPIO.

Live camera feed is served via Flask over VPN.

# Results
Accurate lane tracking in controlled environments.

Real-time object recognition with minimal latency.

Effective obstacle avoidance via ultrasonic sensors.

Smooth motor control for navigation.

Web interface supports remote streaming and monitoring.

# Future Scope
Integrate GPS for navigation over large areas.

Add LiDAR or infrared sensors for improved detection.

Include traffic sign and pedestrian recognition.

Deploy edge AI accelerators for faster inference.

Add telemetry and remote control features to the web UI.

# Team
Amogh Gadad (02FE22BEC006)

Amruta Biradarpatil (02FE22BEC008)

Ashtami Hosapeti (02FE22BEC012)

Naman Timmapur (02FE22BEC039)

Department of Electronics and Communication Engineering

KLE Technological University, Dr. MSSCET, Belagavi
