# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar

import RPi.GPIO as GPIO
import time
import get_blocks
from CW230 import CW230
import atexit
from Tkinter import *

GPIO.setmode(GPIO.BOARD)

diry = 37
pwmy = 35
dirx = 38
pwmx = 40

GPIO.setup(dirx, GPIO.OUT)
GPIO.setup(pwmx, GPIO.OUT)
GPIO.setup(diry, GPIO.OUT)
GPIO.setup(pwmy, GPIO.OUT)

#Define minimum number of steps motor can move
N = 1

#define x and y coordinates, and their default centre value
global x, y
x = 160
y = 100

#define PID values
Kp = 1
Ki = 0
Kd = 0
print "Kp = %d, Ki = %d, Kd = %d"%(Kp, Ki, Kd)

#Define x and y Motors
xMotor = CW230(dirx,pwmx)
yMotor = CW230(diry,pwmy)

#Initialise PID variables
previousErrorX = [0,0]
previousErrorY = [0,0]
Ts = 0.05 #sampling time

#declare Timer Objects
tPID = time.time() #sampling time is 0.05s
tX = time.time() #delay time is 0.001s, toggle time 0.0005s
tY = time.time() #delay time is 0.001s, toggle time 0.0005s

#declare PID variables
PIDx = 0
previousPIDx = 0
PIDy = 0
previousPIDy = 0
#functions for PID compensator

def xCompensator(xError):
    global PIDx, previousPIDx
    a = Kp + Ki*Ts/2 + Kd/Ts
    b = -Kp + Ki*Ts/2 - 2*Kd/Ts
    c = Kd/Ts

    PIDx = previousPIDx + a*xError + b*previousErrorX[1] + c*previousErrorX[0]
    previousPIDx = PIDx
    previousErrorX[0] = previousErrorX[1]
    previousErrorX[1] = xError   
  

def yCompensator(yError):
    global PIDy, previousPIDy
    a = Kp + Ki*Ts/2 + Kd/Ts
    b = -Kp + Ki*Ts/2 - 2*Kd/Ts
    c = Kd/Ts

    PIDy = previousPIDy + a*yError + b*previousErrorY[1] + c*previousErrorY[0]
    previousPIDy = PIDy
    previousErrorY[0] = previousErrorY[1]
    previousErrorY[1] = yError

    return


def calculateX():
    global PIDx
    if abs(160-x) > 2:
        xCompensator(160-x)
    else:
        PIDx = 0
  
    return

def calculateY():
    global PIDy
    if abs(y-100) > 2:
        yCompensator(100-y)
    else:
        PIDy = 0
    
    return
    

blocks = get_blocks.BlockArray(100)


#following function calculates x and y coordinates of the object
def pixy():
    global x, y
    count = get_blocks.pixy_get_blocks(100, blocks)
    if count > 0:
        print 'OUTPUT: [X=%3d Y=%3d]' % (blocks[0].x, blocks[0].y)
        x1 = blocks[0].x
        y1 = blocks[0].y
        x2 = blocks[1].x
        y2 = blocks[1].y
        x = (x1+x2)/2
        y = (y1+y2)/2
    else:
       x = 160
       y = 100
       print 'no objects detected'

    return


def all_done():
    GPIO.cleanup()
    print 'all_done()'
  
atexit.register(all_done)

def show_entry_fields():
   print("First Name: %s\nLast Name: %s" % (e1.get(), e2.get()))

master = Tk()

def mainLoop():
    print("Hello")
    global tPID, tX, tY, xMotor, yMotor
    pixy()
    if(time.time() - tPID >= 0.05):
         calculateX()
         calculateY()
         xMotor.setSteps(PIDx)
         yMotor.setSteps(PIDy)
         tPID = time.time()

    if(time.time() - tX >= xMotor.delay):
         xMotor.toggle()
         tX = time.time()

    if(time.time() - tY >= yMotor.delay):
         yMotor.toggle()
         tY = time.time()
    master.after(1,mainLoop)
    

Label(master, text="Welcome to the Mouse Tracking System!").grid(row=0)
Label(master, text="Enter the coefficients for:").grid(row=1)
Label(master, text="Point1: ").grid(row=2)
Label(master, text="Point2: ").grid(row=3)
Label(master, text="Eq'n would be: 'c1*P1 + c2*P2' ").grid(row=4)

e1 = Entry(master)
e2 = Entry(master)

e1.grid(row=2, column=1)
e2.grid(row=3, column=1)

Button(master, text='Quit', command=master.quit).grid(row=4, column=2, sticky=W, pady=2)
Button(master, text='Enter', command=show_entry_fields).grid(row=4, column=1, sticky=W, pady=4)

master.after(1,mainLoop)
master.mainloop()

