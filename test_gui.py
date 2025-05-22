#!/usr/bin/env python3
import cv2

# Create a blank image
img = cv2.imread("cake_image.jpg")
if img is None:
    print("Error: Could not load image")
    exit()

# Create a window and display the image
cv2.namedWindow("Test Window")
cv2.imshow("Test Window", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
