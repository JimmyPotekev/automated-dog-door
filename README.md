# automated-dog-door

# Automated Dog Door

## Overview
This project is an automated dog door designed to help manage access for dogs to go outside without leaving a door open all the time. The system uses dual cameras and object detection to recognize specific dogs and trigger a servo-powered door mechanism.

## Motivation
I have three dogs (two boxers and a Boston terrier), and my mom finds it difficult to let them out as often as needed for exercise. Leaving the door open for them is not ideal due to heating and air conditioning concerns. This system automates the process and ensures that only the dogs can trigger the door, improving security and efficiency.

## Features
- Dual Raspberry Pi Camera Module 3 for indoor and outdoor object detection.
- YOLOv8 object detection model to recognize only our dogs.
- Motion-activated camera system using distance sensors to reduce power consumption.
- Servo-driven door mechanism powered by a PCA9685 PWM controller for smooth, jitter-free operation.
- Failsafe reconnect logic to handle temporary camera disconnections.
- Automatic door opening and closing based on real-time detection.

## Hardware Components
- Raspberry Pi 5 (primary controller)
- 4 x MG996R servos (for the door panels)
- PCA9685 PWM driver (for better servo control, reducing jitter)
- 2 x Raspberry Pi Camera Module 3 (for object detection)
- 2 x HC-SR04 ultrasonic distance sensors (motion detection to activate cameras only when needed)
- Custom power management board (to regulate and distribute power efficiently)

## Software Stack
- Python 3.11
- OpenCV (for image processing and visualization)
- YOLOv8 (Ultralytics) (for real-time object detection)
- Picamera2 (for interfacing with Raspberry Pi cameras)
- gpiozero (for distance sensor control)
- Adafruit ServoKit (for controlling the PCA9685 PWM driver and servos)

## Power Optimization
To reduce unnecessary power consumption, the system only activates cameras when motion is detected within a certain range. Testing showed that this implementation saves about 20% more power compared to a system where cameras run continuously.

## Installation & Setup
1. Set up Raspberry Pi 5 with Raspberry Pi OS.
2. Enable I2C and Camera interfaces from `raspi-config`.
3. Create and activate a virtual environment:
   ```sh
   python -m venv venv
   source venv/bin/activate
   ```
4. Install dependencies:
   ```sh
   pip install opencv-python picamera2 ultralytics gpiozero adafruit-circuitpython-servokit
   ```
5. Connect hardware components (PCA9685, servos, cameras, distance sensors).
6. Run the main script inside the virtual environment:
   ```sh
   python automated_dog_door.py
   ```

## Potential Improvements
- Implement a user-friendly UI to adjust detection thresholds and sensitivity.
- Add cloud-based monitoring to log usage and provide alerts.
- Improve **detection robustness by using custom-trained YOLO models.



