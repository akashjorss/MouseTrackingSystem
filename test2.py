# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar
import RPi.GPIO as GPIO
import time
import atexit
import numpy as np


GPIO.setmode(GPIO.BOARD)

GPIO.setup(38,GPIO.OUT)
GPIO.setup(40,GPIO.OUT)
GPIO.output(38,True)
GPIO.output(40,True)
pwm_time = time.time()
dir_time = time.time()
pwm_toggle = 1
dir_toggle = 1


while 1:

    if time.time() - pwm_time >= 0.0005:
        pwm_toggle = abs(pwm_toggle - 1)
        GPIO.output(40,pwm_toggle)
        pwm_time = time.time()
    if time.time() - dir_time >= 2:
        dir_toggle = abs(dir_toggle - 1)
        GPIO.output(38,dir_toggle)
        dir_time = time.time()
         

