import cv2
from picamera2 import Picamera2, Picamera2Error
from ultralytics import YOLO
from gpiozero import DistanceSensor
from adafruit_servokit import ServoKit
from time import sleep, time
from total_power_vs_time import PowerMonitor
from concurrent.futures import ProcessPoolExecutor, as_completed
import itertools

# --- Worker function for parallel YOLO inference ---
def process_batch(frames, model_path, imgsz=160):
    """
    Run YOLO inference on a batch of frames.
    Each process loads its own YOLO model.
    """
    model = YOLO(model_path)
    batch_results = []
    for frame in frames:
        results = model(frame, imgsz=imgsz)
        detected = results[0].boxes.cls.tolist()
        batch_results.append((results, detected))
    return batch_results


# --- Servo setup ---
kit = ServoKit(channels=16)
servo1, servo2, servo3, servo4 = kit.servo[0], kit.servo[1], kit.servo[2], kit.servo[3]
for servo in [servo1, servo2, servo3, servo4]:
    servo.set_pulse_width_range(500, 2500)

def open_door():
    for s in [servo1, servo2, servo3, servo4]:
        s.angle = 180
    print("Doors opened")

def close_door():
    for s in [servo1, servo2, servo3, servo4]:
        s.angle = 0
    print("Doors closed")


# --- Camera reconnect helper ---
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


# --- Initialize cameras ---
try:
    inside_camera = Picamera2(0)
    inside_camera.preview_configuration.main.size = (1280, 1280)
    inside_camera.preview_configuration.main.format = "RGB888"
    inside_camera.preview_configuration.align()
    inside_camera.configure("preview")
    inside_available = True
except Picamera2Error:
    print("Error: Inside camera not detected!")
    inside_camera, inside_available = None, False

try:
    outside_camera = Picamera2(1)
    outside_camera.preview_configuration.main.size = (1280, 1280)
    outside_camera.preview_configuration.main.format = "RGB888"
    outside_camera.preview_configuration.align()
    outside_camera.configure("preview")
    outside_available = True
except Picamera2Error:
    print("Error: Outside camera not detected!")
    outside_camera, outside_available = None, False


# --- Sensors and power monitor ---
inside_sensor = DistanceSensor(echo=24, trigger=18)
outside_sensor = DistanceSensor(echo=25, trigger=19)

monitor = PowerMonitor()
monitor.start()

# --- Config ---
MODEL_PATH = "yolov8n_ncnn_model"
objects_to_detect = [0, 16]   # YOLO class IDs
batch_size = 4                # tune based on CPU cores
door_open_time = 0
open_duration = 3
active_camera = None


# --- Main loop ---
with ProcessPoolExecutor() as executor:
    pending = set()
    frame_buffer = []

    while True:
        inside_dist = inside_sensor.distance * 100
        outside_dist = outside_sensor.distance * 100
        print(f"Inside Distance: {inside_dist:.1f} cm, Outside Distance: {outside_dist:.1f} cm")

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
            sleep(0.1)
            if not inside_available:
                inside_camera, inside_available = try_reconnect_camera(0)
            if not outside_available:
                outside_camera, outside_available = try_reconnect_camera(1)
            continue

        # --- Batch up frames ---
        frame_buffer.append(frame)
        if len(frame_buffer) >= batch_size:
            fut = executor.submit(process_batch, frame_buffer, MODEL_PATH)
            pending.add(fut)
            frame_buffer = []

        # --- Collect results as they finish ---
        done, pending = as_completed(pending, timeout=0), pending
        for fut in done:
            try:
                batch_results = fut.result()
                for results, detected_objects in batch_results:
                    object_found = any(obj_id in detected_objects for obj_id in objects_to_detect)
                    if object_found:
                        print(f"Dog detected by {active_camera} camera!")
                        open_door()
                        door_open_time = time()
                    elif time() - door_open_time > open_duration:
                        print("No dog detected, closing doors")
                        close_door()
                        if active_camera == "inside" and inside_available:
                            inside_camera.stop()
                        elif active_camera == "outside" and outside_available:
                            outside_camera.stop()
                        active_camera = None

                    # Debug display
                    annotated = results[0].plot()
                    cv2.imshow("Object Detection", annotated)
            except Exception as e:
                print(f"Error in batch processing: {e}")

        if cv2.waitKey(1) == ord("q"):
            break

# --- Cleanup ---
avg_power = monitor.stop()
print(f"Test completed. Average power: {avg_power:.3f} W")
cv2.destroyAllWindows()
