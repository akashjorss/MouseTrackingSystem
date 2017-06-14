# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar

import thread 
import RPi.GPIO as GPIO
import time
import get_blocks
from StepperMotor import StepperMotor
import atexit

GPIO.setmode(GPIO.BOARD)

xMotorPins = [5,7,8,10] #pins to drive stepper
yMotorPins = [35,37,38,40]



for pinx, piny in zip(xMotorPins, yMotorPins):
    GPIO.setup(pinx,GPIO.OUT)
    GPIO.output(pinx,False)
    GPIO.setup(piny,GPIO.OUT)
    GPIO.output(piny,False)



#Define minimum number of steps motor can move
N = 1

#define x and y coordinates, and their default centre value
global x, y
x = 160
y = 100

#define PID values
Kp = 2
Ki = 0.05
##0.038576
Kd = 1
print "Kp = %d, Ki = %d, Kd = %d"%(Kp, Ki, Kd)

#Define x and y Motors
xMotor = StepperMotor(xMotorPins)
yMotor = StepperMotor(yMotorPins)

#Initialise PID variables
Px = 0
Ix = 0
Dx = 0
previousErrorX = 0
Py = 0
Iy = 0
Dy = 0
previousErrorY = 0

#declare Timer Objects
tPID = time.time() #sampling time is 0.05s
tX = time.time() #delay time is 0.00857s
tY = time.time() #delay time is 0.00857s

#functions for PID compensator

def xCompensator(xError):
        global Px, Ix, Dx, previousErrorX
	Px = Kp*xError
	Ix = Ix + xError*Ki
	Dx = (xError - previousErrorX)*Kd
        print "Ix = %f, Dx = %f"%(Ix, Dx)
        print "Kp = %f, Ki = %f, Kd = %f"%(Kp, Ki, Kd)
        previousErrorX = xError
	return (Px + Ix + Dx)
    

def yCompensator(yError):
        global Py, Iy, Dy, previousErrorY
	Py = Kp*yError
	Iy = Iy + yError*Ki
	Dy = (yError - previousErrorY)*Kd
	print "Py = %f, Iy = %f, Dy = %f"%(Py, Iy, Dy)
	print "Kp = %f, Ki = %f, Kd = %f"%(Kp, Ki, Kd)
        previousErrorY = yError
	return (Py + Iy + Dy)

#define functions for x, y motor control and pixy for multithreading

def calculateX():
    
    if abs(x - 160) > 3:
        PIDx = xCompensator(x - 160)
    else:
        PIDx = 0

##    PIDx = xCompensator(x - 160)
        
    return PIDx*N

def calculateY():

    if abs(y - 100) > 3:
        PIDy = yCompensator(100 - y)
    else:
        PIDy = 0

##    PIDy = yCompensator(100 - y)
    
    return PIDy*N
    

blocks = get_blocks.BlockArray(100)


#following function calculates x and y coordinates of the object
def pixy():
    global x, y
    count = get_blocks.pixy_get_blocks(100, blocks)
    if count > 0:
        print 'OUTPUT: [X=%3d Y=%3d]' % (blocks[0].x, blocks[0].y)
        x = blocks[0].x
        y = blocks[0].y
    else:
       x = 160
       y = 100
##       print 'no objects detected'

    return


def all_done():
    GPIO.cleanup()
    print 'all_done()'
  
atexit.register(all_done)

while 1:
     pixy()
     if(time.time() - tPID >= 0.05):
         xMotor.setSteps(calculateX())
         yMotor.setSteps(calculateY())
         tPID = time.time()

     if(time.time() - tX >= xMotor.delay):
         xMotor.rotate()
         tX = time.time()

     if(time.time() - tY >= yMotor.delay):
         yMotor.rotate()
         tY = time.time()
         



    

