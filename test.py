import RPi.GPIO as GPIO
from StepperMotor import StepperMotor
import time

GPIO.setmode(GPIO.BOARD)

motorPins = [35,37,38,40]

for pin in motorPins:
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)
    print('pin %d initialised!'%(pin))

stepper = StepperMotor(motorPins)

stepper.setSteps(100000)
print('steps initialised')

current_time = time.time()
'Stepper Rotating...'
while(abs(stepper.steps) > 0):
    if(time.time() - current_time >= stepper.delay):
        stepper.rotate()
        current_time = time.time()


print('Rotation finished')


    
    


