##Defines a stepper motor class, just for program coherence for me practicing
##my python object oriented skills
##in this program 1 step = 4 steps because 1 step is too small
##that effect is virtually non-visible
import RPi.GPIO as GPIO
import time

class StepperControl:
    '''class for a 2 phase stepper motor'''

    #define class variables
    motorPins = [0, 0, 0, 0, 0, 0, 0, 0]
    delay = 0.000857 #delay between magnets polarising
    advancedSequence = [[1,0,1,0],
                       [0,1,1,0],
                       [0,1,0,1],
                       [1,0,0,1]]
    reversedSequence = list(advancedSequence)
    reversedSequence.reverse()
    
    def __init__(self):
        print('Stepper Motors Initialised!')

    def __init__(self,motorPins):
        print('Stepper Motors Intitialised!')
        self.motorPins = motorPins

    def setMotorPins(self, motorPins):
        self.motorPins = motorPins
        return

    def setDelay(self, delay):
        self.delay = delay
        return

    def setAdvancedSequence(self, advancedSequence):
        self.advancedSequence = advancedSequence
        reversedSequence = list(advancedSequence)
        reversedSequence.reverse()

    def rotateForwardForward(self, xSteps, ySteps):
        if xSteps < ySteps:
            count = xSteps
        else:
            count = ySteps
        while(count > 0):
            for pins in self.advancedSequence:
                GPIO.output(self.motorPins[0],pins[0])
                GPIO.output(self.motorPins[1],pins[1])
                GPIO.output(self.motorPins[2],pins[2])
                GPIO.output(self.motorPins[3],pins[3])
                GPIO.output(self.motorPins[4],pins[0])
                GPIO.output(self.motorPins[5],pins[1])
                GPIO.output(self.motorPins[6],pins[2])
                GPIO.output(self.motorPins[7],pins[3])
                time.sleep(self.delay)
            count -= 1
        if(xSteps > ySteps):
            while xSteps > 0:
                for pins in self.advancedSequence:
                    GPIO.output(self.motorPins[0],pins[0])
                    GPIO.output(self.motorPins[1],pins[1])
                    GPIO.output(self.motorPins[2],pins[2])
                    GPIO.output(self.motorPins[3],pins[3])
                    time.sleep(self.delay)
                xSteps -= 1
        else:
            while ySteps > 0:
                for pins in self.advancedSequence:
                    GPIO.output(self.motorPins[4],pins[0])
                    GPIO.output(self.motorPins[5],pins[1])
                    GPIO.output(self.motorPins[6],pins[2])
                    GPIO.output(self.motorPins[7],pins[3])
                    time.sleep(self.delay)
                ySteps -= 1
            
        return

    def rotateBackwardForward(self, xSteps, ySteps):
        if xSteps < ySteps:
            count = xSteps
        else:
            count = ySteps
        while(count > 0):
            for rpins, pins in zip(self.reversedSequence, self.advancedSequence):
                GPIO.output(self.motorPins[0],rpins[0])
                GPIO.output(self.motorPins[1],rpins[1])
                GPIO.output(self.motorPins[2],rpins[2])
                GPIO.output(self.motorPins[3],rpins[3])
                GPIO.output(self.motorPins[4],pins[0])
                GPIO.output(self.motorPins[5],pins[1])
                GPIO.output(self.motorPins[6],pins[2])
                GPIO.output(self.motorPins[7],pins[3])
                time.sleep(self.delay)
            count -= 1
        if(xSteps > ySteps):
            while xSteps > 0:
                for pins in self.reversedSequence:
                    GPIO.output(self.motorPins[0],pins[0])
                    GPIO.output(self.motorPins[1],pins[1])
                    GPIO.output(self.motorPins[2],pins[2])
                    GPIO.output(self.motorPins[3],pins[3])
                    time.sleep(self.delay)
                xSteps -= 1
        else:
            while ySteps > 0:
                for pins in self.advancedSequence:
                    GPIO.output(self.motorPins[4],pins[0])
                    GPIO.output(self.motorPins[5],pins[1])
                    GPIO.output(self.motorPins[6],pins[2])
                    GPIO.output(self.motorPins[7],pins[3])
                    time.sleep(self.delay)
                ySteps -= 1

        return
        


    def rotateForwardBackward(self, xSteps, ySteps):
        if xSteps < ySteps:
            count = xSteps
        else:
            count = ySteps
        while(count > 0):
            for pins, rpins in zip(self.advancedSequence, self.reversedSequence):
                GPIO.output(self.motorPins[0],pins[0])
                GPIO.output(self.motorPins[1],pins[1])
                GPIO.output(self.motorPins[2],pins[2])
                GPIO.output(self.motorPins[3],pins[3])
                GPIO.output(self.motorPins[4],rpins[0])
                GPIO.output(self.motorPins[5],rpins[1])
                GPIO.output(self.motorPins[6],rpins[2])
                GPIO.output(self.motorPins[7],rpins[3])
                time.sleep(self.delay)
            count -= 1
        if(xSteps > ySteps):
            while xSteps > 0:
                for pins in self.advancedSequence:
                    GPIO.output(self.motorPins[0],pins[0])
                    GPIO.output(self.motorPins[1],pins[1])
                    GPIO.output(self.motorPins[2],pins[2])
                    GPIO.output(self.motorPins[3],pins[3])
                    time.sleep(self.delay)
                xSteps -= 1
        else:
            while ySteps > 0:
                for pins in self.reversedSequence:
                    GPIO.output(self.motorPins[4],pins[0])
                    GPIO.output(self.motorPins[5],pins[1])
                    GPIO.output(self.motorPins[6],pins[2])
                    GPIO.output(self.motorPins[7],pins[3])
                    time.sleep(self.delay)
                ySteps -= 1

        return


    def rotateBackwardBackward(self, xSteps, ySteps):
        if xSteps < ySteps:
            count = xSteps
        else:
            count = ySteps
        while(count > 0):
            for pins in self.reversedSequence:
                GPIO.output(self.motorPins[0],pins[0])
                GPIO.output(self.motorPins[1],pins[1])
                GPIO.output(self.motorPins[2],pins[2])
                GPIO.output(self.motorPins[3],pins[3])
                GPIO.output(self.motorPins[4],pins[0])
                GPIO.output(self.motorPins[5],pins[1])
                GPIO.output(self.motorPins[6],pins[2])
                GPIO.output(self.motorPins[7],pins[3])
                time.sleep(self.delay)
            count -= 1
        if(xSteps > ySteps):
            while xSteps > 0:
                for pins in self.reversedSequence:
                    GPIO.output(self.motorPins[0],pins[0])
                    GPIO.output(self.motorPins[1],pins[1])
                    GPIO.output(self.motorPins[2],pins[2])
                    GPIO.output(self.motorPins[3],pins[3])
                    time.sleep(self.delay)
                xSteps -= 1
        else:
            while ySteps > 0:
                for pins in self.reversedSequence:
                    GPIO.output(self.motorPins[4],pins[0])
                    GPIO.output(self.motorPins[5],pins[1])
                    GPIO.output(self.motorPins[6],pins[2])
                    GPIO.output(self.motorPins[7],pins[3])
                    time.sleep(self.delay)
                ySteps -= 1

        return


        
    def rotate(self, xSteps, ySteps):

        if xSteps >= 0 and ySteps >= 0:
            self.rotateForwardForward(xSteps, ySteps)
        elif xSteps >= 0 and ySteps < 0:
            self.rotateForwardBackward(xSteps, -ySteps)
        elif xSteps < 0 and ySteps >= 0:
            self.rotateBackwardForward(-xSteps,ySteps)
        else:
            self.rotateBackwardBackward(-xSteps,-ySteps)

        return
    
        
