import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

class CW230:
    '''class for special stepper motor driver'''
    CWPin = 0 #direction pin, 1 and 0
    CPPin = 0#PWM pin, # of rising edges = no. of steps
    dir = 0
    toggles = 0 ## 2 toggles per step, # of toggles
    current_state = 0 #current state of PWM CPPin
    delay = 0.0005 #delay between toggles, controls the speed

    def __init__(self):
        print('stepper motor initialised!')

    def __init__(self, CWPin, CPPin):
        print('Stepper initialised with pins')
        self.CWPin = CWPin
        self.CPPin = CPPin

    def setSteps(self,steps):
        if steps > 0:
            self.dir = 0
        else:
            self.dir = 1
        GPIO.output(self.CWPin,self.dir)
        self.toggles = 2*abs(steps)

    def toggle(self):
        '''basically takes half a step'''
        
        if(self.toggles > 0):
            self.current_state = abs(self.current_state - 1)
            GPIO.output(self.CPPin,self.current_state)
            self.toggles -= 1

            
            
        

    
    
