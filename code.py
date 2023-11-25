try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")

import time
import threading
import atexit
import sys
import termios
import contextlib
import maestro

import imutils
import RPi.GPIO as GPIO
#from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22

#######################

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

class VideoUtils(object):
    @staticmethod
    def live_video(camera_port=0):
        video_capture = cv2.VideoCapture()

        while True:
            ret, frame = video_capture.read()

            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)

        firstFrame = None
        tempFrame = None
        count = 0


        while True:
            grabbed, frame = camera.read()

            if not grabbed:
                break

            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\n Waiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break

        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt

class Turret(object):
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode
    #    self.mh = Adafruit_MotorHAT()
        #atexit.register(self.__turn_off_motors)
        self.sm_x = 0
        #self.sm_x = self.mh.getStepper(200, 1)
        #self.sm_x.setSpeed(5)
        self.current_x_steps = 0

        self.sm_y = 0
        #self.sm_y = self.mh.getStepper(200, 2)
        #self.sm_y.setSpeed(5)
        self.current_y_steps = 0
        # Initialize Maestro servo controller
        self.servo_controller = maestro.Controller()
         # Anti-spam defense variables

        # Store the original position for reference
        self.original_servo_position = 6000  # Center position
        
        self.last_trigger_time = 0
        self.trigger_cooldown = 2  # Cooldown period in seconds
        
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(RELAY_PIN, GPIO.OUT)
        #GPIO.output(RELAY_PIN, GPIO.LOW)

    def calibrate(self):
        print("Please calibrate the tilt of the gun so that it is level. Commands: (w) moves up, (s) moves down. Press (enter) to finish.\n")
        self.__calibrate_y_axis()

        print("Please calibrate the yaw of the gun so that it aligns with the camera. Commands: (a) moves left, (d) moves right. Press (enter) to finish.\n")
        self.__calibrate_x_axis()


        # Store the original position after calibration
        self.original_servo_position = self.servo_controller.getPosition(0)  # Assuming servo is connected to channel 0
        
        print("Calibration finished.")

    def __calibrate_x_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self.sm_x, 5)
                        else:
                            Turret.move_forward(self.sm_x, 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self.sm_x, 5)
                        else:
                            Turret.move_backward(self.sm_x, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __calibrate_y_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self.sm_y, 5)
                        else:
                            Turret.move_backward(self.sm_y, 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self.sm_y, 5)
                        else:
                            Turret.move_forward(self.sm_y, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def motion_detection(self, show_video=False):
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        target_steps_x = (2 * MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2 * MAX_STEPS_Y * (y + h / 2) / v_h) - MAX_STEPS_Y

        print(f"x: {target_steps_x}, y: {target_steps_y}")
        print(f"current x: {self.current_x_steps}, current y: {self.current_y_steps}")

        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()

        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_forward, args=(self.sm_x, 2,))
            else:
                t_x = threading.Thread(target=Turret.move_backward, args=(self.sm_x, 2,))
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_backward, args=(self.sm_x, 2,))
            else:
                t_x = threading.Thread(target=Turret.move_forward, args=(self.sm_x, 2,))

        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))
            else:
                t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
            else:
                t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))

        if not self.friendly_mode:
            if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
                t_fire = threading.Thread(target=Turret.fire)

        t_x.start()
        t_y.start()
        t_fire.start()

        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        Turret.move_forward(self.sm_x, 1)
        Turret.move_forward(self.sm_y, 1)

        print('Commands: Pivot with (a) and (d). Tilt with (w) and (s). Exit with (q)\n')
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self.sm_y, 5)
                        else:
                            Turret.move_backward(self.sm_y, 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self.sm_y, 5)
                        else:
                            Turret.move_forward(self.sm_y, 5)
                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self.sm_x, 5)
                        else:
                            Turret.move_forward(self.sm_x, 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self.sm_x, 5)
                        else:
                            Turret.move_backward(self.sm_x, 5)
                    elif ch == "\n":
                        Turret.fire()

            except (KeyboardInterrupt, EOFError):
                pass
                
    def trigger_pull(self):
        current_time = time.time()

        # Check if enough time has passed since the last trigger pull
        if current_time - self.last_trigger_time >= self.trigger_cooldown:
            # Perform trigger pull action in a separate thread
            threading.Thread(target=self._trigger_pull_thread).start()

            # Update the last trigger time
            self.last_trigger_time = current_time

    def _trigger_pull_thread(self):
        # Perform trigger pull action
        print("Trigger pulled!")
        # Add servo control code here (adjust channel and target values)
        self.servo_controller.setTarget(0, 7000)  # Adjust channel and target values accordingly

        # Delay for a short period (adjust as needed)
        time.sleep(1)

        # Move the turret back to its original position after the delay
        self.move_turret_to_original_position()
  
    def move_turret_to_original_position(self):
        # Move the turret back to its original servo position
        self.servo_controller.setTarget(0, self.original_servo_position)
    
    @staticmethod
    def fire():
        print('fire')
        t.trigger_pull()
       # GPIO.output(RELAY_PIN, GPIO.HIGH)
      #  time.sleep(1)
        #GPIO.output(RELAY_PIN, GPIO.LOW)

    @staticmethod
    def move_forward(motor, steps):
        print('move_forward')
       #motor.step(steps, Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)

    @staticmethod
    def move_backward(motor, steps):
        print('move backward')
        #motor.step(steps, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)

    def __turn_off_motors(self):
        print('turn off')
     #   self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
      #  self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
       # self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        #self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

if __name__ == "__main__":
    t = Turret(friendly_mode=False)

    user_input = input("Choose an input mode: (1) Motion Detection, (2) Interactive\n")

    if user_input == "1":
        t.calibrate()
        if input("Live video? (y, n)\n").lower() == "y":
            t.motion_detection(show_video=True)
        else:
            t.motion_detection()
    elif user_input == "2":
        if input("Live video? (y, n)\n").lower() == "y":
            threading.Thread(target=VideoUtils.live_video).start()
        t.interactive()
    else:
        print("Unknown input mode. Please choose a number (1) or (2)")
