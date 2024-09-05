# write a file that finds the brightest pixel in a opencv video stream 

import cv2
import numpy as np
import time

# Open the video stream
cap = cv2.VideoCapture(4)

while True:
    # Read the frame
    ret, frame = cap.read()
    # Convert the frame to Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # perform a naive attempt to find the (x, y) coordinates of
    # the area of the image with the largest intensity value
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    cv2.circle(frame, maxLoc, 5, (255, 0, 0), 2)

    cv2.imshow("Naive", frame)
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break