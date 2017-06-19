# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar
import RPi.GPIO as GPIO
import time
import atexit

GPIO.setmode(GPIO.BOARD)

GPIO.setup(35,GPIO.OUT)
GPIO.setup(37,GPIO.OUT)
GPIO.output(37,True)
GPIO.output(35,True)
pwm_time = time.time()
dir_time = time.time()
pwm_toggle = 1
dir_toggle = 1
while 1:

    if time.time() - pwm_time >= 0.0005:
        pwm_toggle = abs(pwm_toggle - 1)
        GPIO.output(35,pwm_toggle)
        pwm_time = time.time()
    if time.time() - dir_time >= 1:
        dir_toggle = abs(dir_toggle - 1)
        GPIO.output(37,dir_toggle)
        dir_time = time.time()
         


