import RPi.GPIO as GPIO
import time
import math

servoPin2 = 2
servoPin3 = 3
servoPin4 = 4
servoPin1 = 17 #base

baseServo0 = 8.65

sleepTime = 0.6
delay = 0.005

state1 = 7.2
state2 = 6.2
state3 = 6.2
state4 = 6

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(servoPin1, GPIO.OUT)
    GPIO.setup(servoPin2, GPIO.OUT)
    GPIO.setup(servoPin3, GPIO.OUT)
    GPIO.setup(servoPin4, GPIO.OUT)
    
    global servo1
    servo1 = GPIO.PWM(servoPin1, 50)
    servo1.start(state1)
    
    global servo2
    servo2 = GPIO.PWM(servoPin2, 50)
    servo2.start(state2)
    
    global servo3
    servo3 = GPIO.PWM(servoPin3, 50)
    servo3.start(state3)
    
    global servo4
    servo4 = GPIO.PWM(servoPin4, 50)
    servo4.start(state4)
    
    time.sleep(sleepTime)

def ServoUpdate(servoNum,val):
    global state1
    global state2
    global state3
    global state4
    
    if servoNum == 1:
        if val > state1:
            while state1 < val:
                state1 += 0.1
                servo1.ChangeDutyCycle(state1)
                time.sleep(delay)
        else:
            while state1 > val:
                state1 -= 0.1
                servo1.ChangeDutyCycle(state1)
                time.sleep(delay)
        
    
    if servoNum == 2:
        if val > state2:
            while state2 < val:
                state2 += 0.1
                servo2.ChangeDutyCycle(state2)
                time.sleep(delay)
        else:
            while state2 > val:
                state2 -= 0.1
                servo2.ChangeDutyCycle(state2)
                time.sleep(delay)
                
    if servoNum == 3:
        if val > state3:
            while state3 < val:
                state3 += 0.1
                servo3.ChangeDutyCycle(state3)
                time.sleep(delay)
        else:
            while state3 > val:
                state3 -= 0.1
                servo3.ChangeDutyCycle(state3)
                time.sleep(delay)
                
    if servoNum == 4:
        if val > state4:
            while state4 < val:
                state4 += 0.1
                servo4.ChangeDutyCycle(state4)
                time.sleep(delay)
        else:
            while state4 > val:
                state4 -= 0.1
                servo4.ChangeDutyCycle(state4)
                time.sleep(delay)
        
def getX(z,x1,x2,y1,y2):
    return x2 + (z - y2) * (x1 - x2) / (y1 - y2)

def getY(z,x1,x2,y1,y2):
    return x2 + (z - y2) * (x1 - x2) / (y1 - y2)

def servoUpdatePerCm(val):
    val += 1;
    (x,y) = (0.0, 0.0)
    if val > 23.5 or val < 13:
        return False
    
    if val > 21.2:
        x = getX(val, 10.5, 9.5, 23.5, 21.2)
        y = getY(val, 10.5, 9, 23.5, 21.2)
    
    elif val > 17:
        x = getX(val, 9.5, 9, 21.2, 19)
        y = getY(val, 9, 8, 21.2, 19)        
    
    elif val > 14:
        x = getX(val, 9, 8.8, 19, 16.5)
        y = getY(val, 8, 7, 19, 16.5)
    
    else :
        x = getX(val, 8.8, 8, 16.5, 13)
        y = getY(val, 7, 5, 16.5, 13)
        
    ServoUpdate(2,x)
    ServoUpdate(3,y)
    #gripOpen()
    time.sleep(sleepTime)
    #gripClose()
        
    return True

def updateAngle(angle):
    global baseServo0
    val = baseServo0 - (10*angle)/math.pi
    print("valll = ", val)
    ServoUpdate(1,val)
    time.sleep(sleepTime)
    
def gripOpen():
    ServoUpdate(4, 6)
    time.sleep(sleepTime)
    
def gripClose():
    ServoUpdate(4, 9)
    time.sleep(sleepTime)