import RPi.GPIO as GPIO
from StepperMotor import StepperMotor
import time

GPIO.setmode(GPIO.BOARD)

motorPins = [5,7,8,10]

for pin in motorPins:
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)

step = 0
advancedSequence = [[1,0,1,0],
                    [0,1,1,0],
                    [0,1,0,1],
                    [1,0,0,1]]
currentStep = advancedSequence[0]
t0 = time.time()

options = { 0: [1,0,1,0],
            1: [0,1,1,0],
            2: [0,1,0,1],
            3: [1,0,0,1] }

while 1:
##    print currentStep
    if(time.time() - t0 >= 0.000857):
        if step == 0:
            GPIO.output(motorPins[0],1)                
            GPIO.output(motorPins[1],0)
            GPIO.output(motorPins[2],1)
            GPIO.output(motorPins[3],0)
        elif step == 1:
            GPIO.output(motorPins[0],0)                
            GPIO.output(motorPins[1],1)
            GPIO.output(motorPins[2],1)
            GPIO.output(motorPins[3],0)
        elif step == 2:
            GPIO.output(motorPins[0],0)                
            GPIO.output(motorPins[1],1)
            GPIO.output(motorPins[2],0)
            GPIO.output(motorPins[3],1)
        elif step == 3:
            GPIO.output(motorPins[0],1)                
            GPIO.output(motorPins[1],0)
            GPIO.output(motorPins[2],0)
            GPIO.output(motorPins[3],1)
            
        t0 = time.time()
        step += 1
        step = step%4


    

    
