#use raspberry pi camera module to take a video

import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from cvzone.FaceDetectionModule import FaceDetector
import cv2
import time
GPIO.cleanup()
in3 = 23
in4 = 24
in1 = 22
in2 = 27
factory=PiGPIOFactory()
servopin=14
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
servo=Servo(servopin, pin_factory=factory)
direction=''
GPIO.setwarnings(False)
BUZZER= 26
buzzState = False
GPIO.setup(BUZZER, GPIO.OUT)

def turn_left():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
def stop():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)

def turn_right():
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
stop()

cap = cv2.VideoCapture(0)
frame_rate = 8
prev = 0
GPIO.output(BUZZER,GPIO.HIGH)


detector = FaceDetector()
# h = 480
w = 640
sensi = 50
servo.max()
while True:

    time_elapsed = time.time() - prev
    success, img = cap.read()
    if time_elapsed > 1./frame_rate:
        prev = time.time()

        # Do something with your image here.
        img, bboxs = detector.findFaces(img)

        if bboxs:
            GPIO.output(BUZZER,GPIO.HIGH)
            # bboxInfo - "id","bbox","score","center"
            center = bboxs[0]["center"]
            cv2.circle(img, center, 5, (255, 0, 255), cv2.FILLED)
            if center[0] < (w/2 - sensi) :
                GPIO.output(BUZZER,GPIO.LOW)
                turn_right()
                time.sleep(0.025)
                stop()

            if  center[0] > (w/2 + sensi) :
                GPIO.output(BUZZER,GPIO.LOW)
                turn_left()
                time.sleep(0.025)
                stop()


        cv2.imshow("Image", img)
    k=cv2.waitKey(1)
    if k == ord('s'):
        servo.mid()
    if k == ord('r'):
        servo.max()
    if k == ord('q'):
        GPIO.output(BUZZER,GPIO.LOW)
        break;
cap.release()
GPIO.cleanup()
cv2.destroyAllWindows()