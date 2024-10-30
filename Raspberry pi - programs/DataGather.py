import cv2
import numpy as np
from ultralytics import YOLO
import time

# Load YOLOv8 model
model = YOLO("best.pt")

def normalize_brightness(image):
    img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
    img_normalized = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return img_normalized

def rotate_image(image, angle):
    (h, w) = image.shape[:2]
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated

def capture_and_save():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        frame = normalize_brightness(frame)
        frame = rotate_image(frame, -90)  # Rotate image 90 degrees to the right
        results = model(frame)
        if len(results) > 0:
            objects = results[0].boxes.data.cpu().numpy()  # Extract the results as numpy array
            
            # Filter objects with confidence score > 50%
            objects = [obj for obj in objects if obj[4] > 0.3]

            if len(objects) > 0:
                # Select the object with the highest confidence score
                best_object = max(objects, key=lambda obj: obj[4])
                xmin, ymin, xmax, ymax, confidence, class_id = best_object
                xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
                label = model.names[int(class_id)]

                # Draw bounding box and label on the image
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} ({confidence:.2f})", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Create a unique filename
                filename = f"detected_{int(time.time())}.jpg"
            else:
                # Create a unique filename
                filename = f"no_detection_{int(time.time())}.jpg"

            # Save the image
            cv2.imwrite(filename, frame)
            print(f"Image saved as {filename}")

    cap.release()

if __name__ == "__main__":
    while True:
        input("Press Enter to capture and save an image...")
        capture_and_save()
