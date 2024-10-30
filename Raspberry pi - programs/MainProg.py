import time
import serial
import cv2
import numpy as np
from gpiozero import LED, Button
from ultralytics import YOLO

# Setup GPIO pins
green_led = LED(27)
red_led = LED(17)
button = Button(4)

# Initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600)

# Load YOLOv8 model
model = YOLO("best.pt")

# LED initial state
green_led.on()
red_led.off()

def classify_object(area, label):
    if label == "bottle":  # Changed from "plastic_bottle"
        if area < 30000:
            return "small", 0.08
        elif area < 43000:
            return "medium", 0.1
        elif area < 70000:
            return "large", 0.125
        else:
            return "extra_large", 0.24
    elif label == "can":
        if area < 25000:
            return "small", 0.115
        else:
            return "large", 0.165

def normalize_brightness(image):
    img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    img_normalized = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return img_normalized

def rotate_image(image, angle):
    (h, w) = image.shape[:2]
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated

def capture_and_classify():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        frame = normalize_brightness(frame)
        frame = rotate_image(frame, -90)  # Rotate image 90 degrees to the right
        results = model(frame)
        if len(results) > 0:
            objects = results[0].boxes.data.cpu().numpy()  # Extract the results as a numpy array
            
            if objects.size > 0:
                best_object = objects[np.argmax(objects[:, 4])]  # Get the object with the highest confidence score
                xmin, ymin, xmax, ymax, confidence, class_id = best_object
                xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
                
                # Calculate width and height of the bounding box
                width = xmax - xmin
                height = ymax - ymin
                
                # Calculate the area of the bounding box
                area = width * height
                
                label = model.names[int(class_id)]
                print(f"Detected {label} with area {area:.2f} pixels")  # Debug print
                cap.release()
                return classify_object(area, label), label

    cap.release()
    return None, None

def check_arduino_ready():
    ser.write(b"check_ready\n")
    while True:
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            return response == "ready"

while True:
    if button.is_pressed:
        green_led.off()
        red_led.on()
        time.sleep(0.5)

        if check_arduino_ready():
            classification, label = capture_and_classify()
            if classification:
                size_label, points = classification
                message = f"{label},{size_label},{points}\n"
                print(f"Sending to Arduino: {message}")  # Debug print
                ser.write(message.encode())
            else:
                ser.write(b"no_object\n")

            while True:
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    if response == "done":
                        green_led.on()
                        red_led.off()
                        break
        else:
            green_led.on()
            red_led.off()
    time.sleep(0.1)