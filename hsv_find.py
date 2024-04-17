import cv2 as cv
import numpy as np

# Callback function for trackbar
def update_blue_lower(value):
    global lower_blue
    lower_blue[0] = value

def update_blue_upper(value):
    global upper_blue
    upper_blue[0] = value

# Initialize webcam
cap = cv.VideoCapture(0)

# Create a window
cv.namedWindow('frame')

# Initialize the trackbars
cv.createTrackbar('Lower Hue', 'frame', 0, 179, update_blue_lower)
cv.createTrackbar('Upper Hue', 'frame', 0, 179, update_blue_upper)

# Set initial values for blue color range
lower_blue = np.array([80, 50, 50])
upper_blue = np.array([120, 255, 255])

while True:
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame, frame, mask=mask)

    # Display the frame, mask, and result
    cv.imshow('frame', frame)
    cv.imshow('mask', mask)
    cv.imshow('res', res)

    # Exit when 'esc' key is pressed
    if cv.waitKey(1) & 0xFF == 27:
        break

# Release resources and close windows
cap.release()
cv.destroyAllWindows()
