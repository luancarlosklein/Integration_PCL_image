from cvlib.object_detection import YOLO
import cvlib as cv
from cvlib.object_detection import draw_bbox
import cv2
import matplotlib.pyplot as plt
# Create a VideoCapture object to access the webcam
cap = cv2.VideoCapture(0)  # 0 represents the default camera (index 0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Create a window to display the webcam feed
cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)


while True:
    # Read the current frame from the webcam
    ret, frame = cap.read()
    bbox, label, conf = cv.detect_common_objects(frame, confidence=0.25, model='yolov4-tiny')
    
    frame = draw_bbox(frame, bbox, label, conf)

    # Display the frame in the "Webcam" window
    cv2.imshow("Webcam", frame)

    # Wait for the 'q' key to be pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close all windows
cap.release()
cv2.destroyAllWindows()