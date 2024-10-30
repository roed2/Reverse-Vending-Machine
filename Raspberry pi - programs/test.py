import cv2
import numpy as np
from ultralytics import YOLO
import matplotlib.pyplot as plt

# Load YOLOv8 model
model = YOLO("best.pt")

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

def capture_and_check_size():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        #frame = normalize_brightness(frame)
        frame = rotate_image(frame, -90)  # Rotate image 90 degrees to the right
        results = model(frame)
        if len(results) > 0:
            objects = results[0].boxes.data.cpu().numpy()  # Extract the results as a numpy array
            
            for obj in objects:
                xmin, ymin, xmax, ymax, confidence, class_id = obj
                xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
                
                # Calculate width and height of the bounding box
                width = xmax - xmin
                height = ymax - ymin
                
                # Calculate the area of the bounding box
                area = width * height
                
                label = model.names[int(class_id)]
                print(f"Detected {label} with area {area:.2f} pixels")

                # Draw bounding box and label on the image
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} ({confidence:.2f})", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Convert BGR image to RGB for Matplotlib
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        plt.imshow(frame_rgb)
        plt.title("Detections")
        plt.axis('off')  # Hide axes
        plt.show()

    cap.release()

if __name__ == "__main__":
    while True:
        input("Press Enter to capture and check object size...")
        capture_and_check_size()
