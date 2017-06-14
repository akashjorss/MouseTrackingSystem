##Defines a stepper motor class, just for program coherence for me practicing
##my python object oriented skills
##in this program 1 step = 4 steps because 1 step is too small
##that effect is virtually non-visible
import RPi.GPIO as GPIO
import time

class StepperMotor:
    '''class for a 2 phase stepper motor'''

    #define class variables
    motorPins = [0, 0, 0, 0]
    currentStep = [0, 0, 0, 0]
    steps = 0 #Steps to be rotated
    delay = 0.000857 #delay between magnets polarising
    advancedSequence = [[1,0,1,0],
                       [0,1,1,0],
                       [0,1,0,1],
                       [1,0,0,1]]
    reversedSequence = list(advancedSequence)
    reversedSequence.reverse()
    
    def __init__(self):
        print('Stepper Motor Initialised!')

    def __init__(self,motorPins):
        print('Stepper Motor Intitialised!')
        self.motorPins = motorPins
        self.currentStep = self.advancedSequence[0]
        return

    def setMotorPins(self, motorPins):
        self.motorPins = motorPins
        GPIO.output(self.motorPins[0],1)                
        GPIO.output(self.motorPins[1],0)
        GPIO.output(self.motorPins[2],1)
        GPIO.output(self.motorPins[3],0)
        return

    def setSteps(self, steps):
        self.steps = steps

    def setDelay(self, delay):
        self.delay = delay
        return

    def setAdvancedSequence(self, advancedSequence):
        self.advancedSequence = advancedSequence
        reversedSequence = list(advancedSequence)
        reversedSequence.reverse()

    def rotateNextForward(self):
        if(self.steps > 0):
            if(self.currentStep == self.advancedSequence[0]):
                self.currentStep = self.advancedSequence[1]
            elif(self.currentStep == self.advancedSequence[1]):
                self.currentStep = self.advancedSequence[2]
            elif(self.currentStep == self.advancedSequence[2]):
                self.currentStep = self.advancedSequence[3]
            elif(self.currentStep == self.advancedSequence[3]):
                self.currentStep = self.advancedSequence[0]

            print('Current Step = ')
            print(self.currentStep)
            GPIO.output(self.motorPins[0],self.currentStep[0])                
            GPIO.output(self.motorPins[1],self.currentStep[1])
            GPIO.output(self.motorPins[2],self.currentStep[2])
            GPIO.output(self.motorPins[3],self.currentStep[3])

            self.steps -= 1
            
        return
            

    def rotateNextBackward(self):
        if(self.steps < 0):
            if(self.currentStep == self.reversedSequence[0]):
                self.currentStep = self.reversedSequence[1]
            elif(self.currentStep == self.reversedSequence[1]):
                self.currentStep = self.reversedSequence[2]
            elif(self.currentStep == self.reversedSequence[2]):
                self.currentStep = self.reversedSequence[3]
            elif(self.currentStep == self.reversedSequence[3]):
                self.currentStep = self.reversedSequence[0]

            GPIO.output(self.motorPins[0],self.currentStep[0])                
            GPIO.output(self.motorPins[1],self.currentStep[1])
            GPIO.output(self.motorPins[2],self.currentStep[2])
            GPIO.output(self.motorPins[3],self.currentStep[3])

            self.steps += 1
            
        return
        
    def rotate(self):
        '''rotates the motor in required number of steps
           if n > 0, direction is forward and vice versa'''
        if self.steps > 0:
            self.rotateNextForward()
        else:
            self.rotateNextBackward()

        print('Step = %d'%(self.steps))

        
        
        
        
    
        

        
