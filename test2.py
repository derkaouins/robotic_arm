import servoMotor as sm
import time
#import RPi.GPIO as GPIO
import math

link1 = 8.1
link2 = 8.2
d = 0
delta = 5

#GPIO.setmode(GPIO.BCM)
sm.setup()
#time.sleep(3)

def angle1ServoUpdate(angle):
    val = 10.5 - angle * 5 / (math.pi / 2)
    sm.ServoUpdate(2, val)

def angle2ServoUpdate(angle):
    val = 11 - angle * (11-5.8) / (math.pi / 2)
    sm.ServoUpdate(3, val)

def updatePosition(length):
    angle1 = math.acos((((link1**2)+(length**2)-(link2**2))/(2*link1*length)))
    angle2 = math.pi - angle1 - math.acos((((link1**2)+(link2**2)-(length**2))/(2*link1*link2)))
    angle1ServoUpdate(angle1)
    angle2ServoUpdate(angle2)
    
updatePosition(12)
    