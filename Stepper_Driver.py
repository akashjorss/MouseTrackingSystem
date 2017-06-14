import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

StepPins = [3,4,14,15]





Seq = [[1,0,1,0],
       [0,1,1,0],
       [0,1,0,1],
       [1,0,0,1]]



for pin in StepPins:
    print ("Setup Pins")
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)
a = 0
while a in range(5000):

    for i in range(4):
        for j in range(4):
            if Seq[i][j] == 1:
                GPIO.output(StepPins[j],GPIO.HIGH)
            else:
                GPIO.output(StepPins[j],GPIO.LOW)
        time.sleep(0.000857)
    
   
GPIO.cleanup()
                
    
