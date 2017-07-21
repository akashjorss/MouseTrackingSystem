# Mouse Tracking System
# Developed by Akash Malhotra
# Other members on this Project: Tri Quang and Sumit Lulekar

import RPi.GPIO as GPIO
import time
from CW230 import CW230
import atexit
import pyglet
import sys


step = 20
x = 0
previous_x = 0
y = 0
previous_y = 0


GPIO.setmode(GPIO.BOARD)

dirx = 37
pwmx = 35
diry = 38
pwmy = 40

GPIO.setup(dirx, GPIO.OUT)
GPIO.setup(pwmx, GPIO.OUT)
GPIO.setup(diry, GPIO.OUT)
GPIO.setup(pwmy, GPIO.OUT)


#Define x and y Motors
xMotor = CW230(dirx,pwmx)
yMotor = CW230(diry,pwmy)

def all_done():
    GPIO.cleanup()
    print 'all_done()'
  
atexit.register(all_done)


class ResponsiveWindow(pyglet.window.Window):
    def on_key_press(self,symbol,modifier):
        global x, y, previous_x, previous_y
        print str(symbol)+' key pressed'
        if str(symbol) == '65362':
            x += step
        elif str(symbol) == '65364':
            x -= step
        if str(symbol) == '65361':
            y -= step
        if str(symbol) == '65363':
            y += step
        sys.stdout.flush()
 
        xMotor.setSteps(x-previous_x)
        yMotor.setSteps(y-previous_y)
        previous_x = x
        previous_y = y
            

def main(value):   

    xMotor.toggle()
    yMotor.toggle()
        


window = ResponsiveWindow()
label = pyglet.text.Label('Welcome to Mouse Tracking System :)',
                          font_name='Times New Roman',
                          font_size=24,
                          x=window.width//2, y=window.height//2,
                          anchor_x='center', anchor_y='center')

@window.event
def on_draw():
    pass
##    window.clear()
##    label.draw()
pyglet.clock.schedule_interval(main, xMotor.delay)
pyglet.app.run()

 
         


