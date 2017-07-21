# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar

import RPi.GPIO as GPIO
import time
import get_blocks
from CW230 import CW230
import atexit
import cmath
import keyboard



GPIO.setmode(GPIO.BOARD)

diry = 37
pwmy = 35
dirx = 38
pwmx = 40

GPIO.setup(diry, GPIO.OUT)
GPIO.setup(pwmy, GPIO.OUT)
GPIO.setup(dirx, GPIO.OUT)
GPIO.setup(pwmx, GPIO.OUT)


#define x and y coordinates, and their default centre value
x = 160
y = 100

x1 = 0
x2 = 0
y1 = 0
y2 = 0
x_ref = 170
y_ref = 65

#define PID values
Kp = 1
Ki = 0
Kd = 0

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
displayTime = time.time()

#declare PID variables
PIDx = 0
previousPIDx = 0
PIDy = 0
previousPIDy = 0
#functions for PID compensator

def inputValues():
    global x_ref, y_ref
    x_ref = input("Enter x_ref = ")
    y_ref = input("Enter y_ref = ")
    return

##keyboard.add_hotkey('a',inputValues())

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
    if abs(x_ref - x) > 2:
        xCompensator(x_ref - x)
    else:
        PIDx = 0
    
    return

def calculateY():
    global PIDy
    if abs(y-y_ref) > 2: 
        yCompensator(y_ref-y)
    else:
        PIDy = 0

    return
    


blocks = get_blocks.BlockArray(100)


#following function calculates x and y coordinates of the object
def pixy():
    global x, y, x1, x2, y1, y2, x_ref, y_ref, displayTime
    count = get_blocks.pixy_get_blocks(100, blocks)
     
    if count == 2 :        
        #print 'OUTPUT: [X1=%3d Y1=%3d]' % (blocks[0].x, blocks[0].y)
        #print 'OUTPUT: [X2=%3d Y2=%3d]' % (blocks[1].x, blocks[1].y)

        x1 = blocks[0].x
        y1 = blocks[0].y
        x2 = blocks[1].x
        y2 = blocks[1].y

        x = (x1+x2)/2
        y = (y1+y2)/2 

    elif count == 1:
      # print('One object detected')
        x = blocks[0].x
        y = blocks[0].y

        
    else:      
       x = x_ref
       y = y_ref       
       #print('no objects detected')

    if (time.time() - displayTime >= 0.5):  
        print('no. of signatures = ', count)
        print('x = ', x)
        print('y = ', y)
        displayTime = time.time()

        
    return


def all_done():
##    plt.plot(myListX, myListY)
##    plt.show()
    GPIO.cleanup()
    print('all_done()')
  
atexit.register(all_done)

while 1:
     if keyboard.is_pressed(' '):
         inputValues()
               
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











##def search(count, blocks, element):
##    for i in range(0, count):
##        if(blocks[i].signature == element):
##            return i
##    return -1
##
##def setXY(count, blocks):
##    global x1, x2, y1, y2
##    sig = []
##    for i in range(0,count):
##        sig.append(blocks[i].signature)
##    a = search(count,sig,1)
##    b = search(count,sig,2)
##    c = search(count,sig,3)
##    d = search(count,sig,4)
##    if not(a == -1 or c == -1):
##        x1 = blocks[a].x
##        y1 = blocks[a].y
##        x2 = blocks[c].x
##        y2 = blocks[c].y
##    elif not(b == -1 or d == -1):
##        x1 = blocks[b].x
##        y1 = blocks[b].y
##        x2 = blocks[d].x
##        y2 = blocks[d].y
##    else:
##        x1 = 160
##        y1 = y_ref
##        x2 = 160
##        y2 = y_ref
##    return


##        if x2 - x1 == 0:
##            m = 0
##        else:
##            m = (y2 - y1)/(x2 - x1)
##        if m > 0:
##            x_ref = 178
##            y_ref = 63
##        elif (x2 - x1) == 0:
##            x_ref = 155
##            y_ref = 65
##        else:
##            x_ref = 178
##            y_ref = 63       
 
         
        #setXY(count,blocks)
##        i = search(count,blocks,2)
##        j = search(count,blocks,3)
##        if not(i == -1):
##            x1 = blocks[i].x
##            y1 = blocks[i].y
##        if not(j == -1):
##            x2 = blocks[j].x
##            y2 = blocks[j].y
##        #x_ref = (x1+x2)/2
##        #y_ref = (y1+y2)/2
##        #k = search(count,blocks,1)
##        if not(k == -1):
##            x = blocks[k].x
##            y = blocks[k].y

##
##    elif count == 1:
##       print('one object detected')
##       x = blocks[0].x
##       y = blocks[0].y

        
##        x1 = blocks[0].x
##        y1 = blocks[0].y
##        x2 = blocks[1].x
##        y2 = blocks[1].y
##        x3 = (x1+x2)/2
##        y3 = (y1+y2)/2
##        m = -1/((y2-y1)/(x2-x1))
##        c = -x3*m + y3
##        a1 = (1+m**2)
##        a2 = (-2*x3+2*m*c-2*m*y3)
##        a3 = x3*x3+c*c+y3*y3-2*y3*c -0
##        d = a2**2 - 4*a1*a3
##        sol1 = (-a2-cmath.sqrt(d))/(2*a1)
##        sol2 = (-a2+cmath.sqrt(d))/(2*a1)
##        x = sol2.real
##        print('m = ', m)
##        print('c = ', c)
##        y = m*x + c
##        x = x3
##        y = y3
        
##        x = 0.5*x1+0.5*x2 #modify this!!!!!!! :D :P 
##        y = 0.5*y1+0.5*y2
##    elif count == 1:
##        print('One signature detected')
##        if blocks[0].signature == 1:
##            x1 = blocks[0].x
##            y1 = blocks[0].y
##        elif blocks[0].signature == 2:
##            x2 = blocks[0].x
##            y2 = blocks[0].y
##        x = 0.5*x1+0.5*x2 #modify this!!!!!!! :D :P 
##        y = 0.5*y1+0.5*y2

#Following function transforms the coordinate system so the origin is shifted to the midpoint
#of the two markers
##def rotate(x1,y1,x2,y2):
##    global x,y
##    x = (x1+x2)/2
##    y = (y1+y2)/2
##    v = [x2 - x1, y2 - y1, 0]
##    r = [0,1,0]
##    w = np.cross(v, r)
##    w = w/(np.linalg.norm(w))
##    w_matrix = [[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]]
##    theta = np.arccos(np.dot(v,r)/np.linalg.norm(v))
##    C = np.sin(theta)*w_matrix
##    #print(np.multiply(np.sin(theta),w_matrix))
##    Rot_mat = np.eye(3) + np.multiply(np.sin(theta),w_matrix) + (1 - np.cos(theta))*np.multiply(w_matrix,w_matrix)
##    #Rot_mat = np.exp(theta*w_matrix)
##    B = [x, y, 0]
##    print(Rot_mat)
##    print(np.transpose(B))
##    X = np.dot(Rot_mat,np.transpose(B))
##    print(X)
##    time.sleep(0.5)
##    x = X[0]
##    y = X[1]
##    return


    

