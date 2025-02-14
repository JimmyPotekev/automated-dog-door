import cv2
from picamera2 import Picamera2, Picamera2Error
from ultralytics import YOLO
from gpiozero import DistanceSensor
from adafruit_servokit import ServoKit
from time import sleep, time
from total_power_vs_time import PowerMonitor


# Function to attempt reconnecting a camera
def try_reconnect_camera(camera_index):
    try:
        cam = Picamera2(camera_index)
        cam.preview_configuration.main.size = (1280, 1280)
        cam.preview_configuration.main.format = "RGB888"
        cam.preview_configuration.align()
        cam.configure("preview")
        print(f"Camera {camera_index} reconnected successfully!")
        return cam, True
    except Picamera2Error:
        return None, False


# Setup servo control
kit = ServoKit(channels=16)
servo1 = kit.servo[0]  # Inside left
servo2 = kit.servo[1]  # Inside right
servo3 = kit.servo[2]  # Outside left
servo4 = kit.servo[3]  # Outside right

# Set pulse width range for all servos
for servo in [servo1, servo2, servo3, servo4]:
    servo.set_pulse_width_range(500, 2500)


def open_door():
    servo1.angle = 180
    servo2.angle = 180
    servo3.angle = 180
    servo4.angle = 180
    print("Doors opened")


def close_door():
    servo1.angle = 0
    servo2.angle = 0
    servo3.angle = 0
    servo4.angle = 0
    print("Doors closed")


# Attempt to initialize the inside camera
try:
    inside_camera = Picamera2(0)
    inside_camera.preview_configuration.main.size = (1280, 1280)
    inside_camera.preview_configuration.main.format = "RGB888"
    inside_camera.preview_configuration.align()
    inside_camera.configure("preview")
    inside_available = True
except Picamera2Error:
    print("Error: Inside camera not detected!")
    inside_camera = None
    inside_available = False

# Attempt to initialize the outside camera
try:
    outside_camera = Picamera2(1)
    outside_camera.preview_configuration.main.size = (1280, 1280)
    outside_camera.preview_configuration.main.format = "RGB888"
    outside_camera.preview_configuration.align()
    outside_camera.configure("preview")
    outside_available = True
except Picamera2Error:
    print("Error: Outside camera not detected!")
    outside_camera = None
    outside_available = False

# Initialize motion sensors
inside_sensor = DistanceSensor(echo=24, trigger=18)
outside_sensor = DistanceSensor(echo=25, trigger=19)

# Start monitoring power consumption
monitor = PowerMonitor()
monitor.start()

# Load the YOLO model
model = YOLO("yolov8n_ncnn_model")

# List of class IDs we want to detect
objects_to_detect = [0, 16]

# Track which camera is active (None at start)
active_camera = None

door_open_time = 0
open_duration = 3  # Keep the door open for 3 seconds after detection

while True:
    inside_dist = inside_sensor.distance * 100
    outside_dist = outside_sensor.distance * 100
    print(f"Inside Distance: {inside_dist} cm, Outside Distance: {outside_dist} cm")

    if active_camera is None:
        if inside_dist < 10 and inside_available:
            print("Turning ON Inside Camera")
            inside_camera.start()
            active_camera = "inside"
        elif outside_dist < 10 and outside_available:
            print("Turning ON Outside Camera")
            outside_camera.start()
            active_camera = "outside"

    if active_camera == "inside" and inside_available:
        frame = inside_camera.capture_array()
    elif active_camera == "outside" and outside_available:
        frame = outside_camera.capture_array()
    else:
        sleep(0.1)  # If no active camera, wait to reduce CPU usage

        # Attempt to reconnect missing cameras
        if not inside_available:
            inside_camera, inside_available = try_reconnect_camera(0)
        if not outside_available:
            outside_camera, outside_available = try_reconnect_camera(1)

        continue

    # Run object detection
    results = model(frame, imgsz=160)
    detected_objects = results[0].boxes.cls.tolist()

    # Check for dog detection
    object_found = any(obj_id in detected_objects for obj_id in objects_to_detect)

    if object_found:
        print(f"Dog detected by {active_camera} camera!")
        open_door()
        door_open_time = time()  # Record time when door opens

    elif time() - door_open_time > open_duration:
        print(f"No dog detected, closing doors")
        close_door()
        if active_camera == "inside" and inside_available:
            inside_camera.stop()
        elif active_camera == "outside" and outside_available:
            outside_camera.stop()
        active_camera = None

    # Debugging display (disable when installed)
    annotated_frame = results[0].plot()
    cv2.imshow("Object Detection", annotated_frame)

    # Exit condition for testing and debugging
    if cv2.waitKey(1) == ord("q"):
        break

# Outputs average power usage from run time
average_power = monitor.stop()
print(f"Test completed. Average power: {average_power:.3f} W")

# Cleanup
cv2.destroyAllWindows()
