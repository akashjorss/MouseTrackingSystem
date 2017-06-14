# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar
import RPi.GPIO as GPIO
import time
import get_blocks
from StepperControl import StepperControl
import atexit

GPIO.setmode(GPIO.BOARD)

MotorPins = [5,7,8,10,35,37,38,40]



for pin in MotorPins:
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)

MotorSystem = StepperControl(MotorPins)

while 1:

    MotorSystem.rotate(2000,0)
##    MotorSystem.rotate(-400,0)

