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

# Set up webcam with custom resolution
cap_width = 640  # Adjust as needed
cap_height = 480  # Adjust as needed
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)

# Adjust these values based on your camera and environment
min_area = 100  # Adjusted min_area
max_area = 5000
threshold = 25  # Adjust the threshold based on your environment

# Initialize background subtraction
fgbg = cv2.createBackgroundSubtractorMOG2()

# Define delay_x and delay_y
delay_x = 0.01  # Adjust the delay based on your stepper motor and requirements
delay_y = 0.01

# Set screen size to 400x240
screen_width = 400
screen_height = 240

# Set the threshold for the centroid's position
centroid_threshold_x = 0.4  # Adjust based on your preference
centroid_threshold_y = 0.4  # Adjust based on your preference

while True:
    ret, frame = cap.read()

    # Apply background subtraction
    fgmask = fgbg.apply(frame)

    # Threshold the foreground mask
    _, thresh = cv2.threshold(fgmask, threshold, 255, cv2.THRESH_BINARY)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Assuming only one person is present, use the largest contour
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if min_area < area < max_area:
            # Calculate the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the centroid of the bounding box
            cx = x + w // 2
            cy = y + h // 2

            # Your logic for aiming the turret using stepper motors
            # ...

            # Move stepper motors based on the calculated centroid
            move_stepper_x(cx, delay_x)
            move_stepper_y(cy, delay_y)

            # Check if the humanoid is around the middle of the screen
            if (screen_width * (1 - centroid_threshold_x) < cx < screen_width * (1 + centroid_threshold_x)) and \
               (screen_height * (1 - centroid_threshold_y) < cy < screen_height * (1 + centroid_threshold_y)):
                # Your logic for firing the Nerf gun
                shoot()
                print("Humanoid detected in the shooting zone")

            # Draw a green square around the bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            print("No humanoid detected")

    # Resize the frame to fit the specified screen size
    resized_frame = cv2.resize(frame, (screen_width, screen_height))

    # Display the resized frame on the screen
    cv2.imshow("Live Video Feed", resized_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and cleanup GPIO
cap.release()
cv2.destroyAllWindows()
servo.stop()
GPIO.cleanup()
