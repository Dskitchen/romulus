#!/usr/bin/env python3
import cv2
import numpy as np

def nothing(x):
    pass

# Load an image (replace with a saved image from your camera)
image = cv2.imread("cake_image.jpg")  # Replace with your image path
if image is None:
    print("Error: Could not load image")
    exit()

# Resize for easier viewing
image = cv2.resize(image, (640, 480))

# Convert to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window
cv2.namedWindow("Trackbars")

# Create trackbars for HSV tuning
cv2.createTrackbar("Hue Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("Hue Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("Sat Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Sat Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Val Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Val Max", "Trackbars", 255, 255, nothing)

while True:
    h_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
    h_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
    s_min = cv2.getTrackbarPos("Sat Min", "Trackbars")
    s_max = cv2.getTrackbarPos("Sat Max", "Trackbars")
    v_min = cv2.getTrackbarPos("Val Min", "Trackbars")
    v_max = cv2.getTrackbarPos("Val Max", "Trackbars")
    
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)
    
    cv2.imshow("Original", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"HSV Range: Hue [{h_min}, {h_max}], Sat [{s_min}, {s_max}], Val [{v_min}, {v_max}]")
        break

cv2.destroyAllWindows()
