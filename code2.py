import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up GPIO for stepper motors and servo motor
GPIO_PIN_X = 17  # Example pin number for X-axis motor
GPIO_PIN_Y = 18  # Example pin number for Y-axis motor
GPIO_PIN_SERVO = 23  # Example pin number for the servo motor

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN_X, GPIO.OUT)
GPIO.setup(GPIO_PIN_Y, GPIO.OUT)
GPIO.setup(GPIO_PIN_SERVO, GPIO.OUT)

# Create servo motor object
servo = GPIO.PWM(GPIO_PIN_SERVO, 50)  # 50 Hz frequency

# Define functions for controlling stepper motors and servo motor
def move_stepper_x(steps, delay):
    print("Rotate on X")

def move_stepper_y(steps, delay):
    print("Rotate on Y")

def shoot():
    print("FIRE!")

# Set up webcam
cap = cv2.VideoCapture(0)  # 0 indicates the default camera

# Adjust these values based on your camera and environment
min_area = 500
max_area = 5000
threshold = 25  # Adjust the threshold based on your environment

# Initialize background subtraction
fgbg = cv2.createBackgroundSubtractorMOG2()

while True:
    ret, frame = cap.read()

    # Apply background subtraction
    fgmask = fgbg.apply(frame)

    # Threshold the foreground mask
    _, thresh = cv2.threshold(fgmask, threshold, 255, cv2.THRESH_BINARY)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Calculate the centroid of the first contour (assuming only one person is present)
        M = cv2.moments(contours[0])
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Your logic for aiming the turret using stepper motors
        # ...

        # Move stepper motors based on the calculated centroid
        move_stepper_x(cx, delay_x)
        move_stepper_y(cy, delay_y)

        # Your logic for firing the Nerf gun
        shoot()
        print("Movement detected")
    else:
        print("Movement lost")

    # Display the frame
    cv2.imshow("Frame", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam, close all windows, and cleanup GPIO
cap.release()
cv2.destroyAllWindows()
servo.stop()
GPIO.cleanup()
