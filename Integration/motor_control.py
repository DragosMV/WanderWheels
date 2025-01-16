import RPi.GPIO as GPIO
import time
from time import sleep

# Define Board pin
RFD = 31
RFP = 33
LFP = 35
LFD = 37
RBD = 36
RBP = 32
LBD = 10
LBP = 12

# GPIO set up
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(RFD, GPIO.OUT)  # Right front direction
GPIO.setup(RFP, GPIO.OUT)  # Right front PWM(speed)
GPIO.setup(LFD, GPIO.OUT)  # Left front direction
GPIO.setup(LFP, GPIO.OUT)  # left ~
GPIO.setup(RBD, GPIO.OUT)  # Right Behind direction
GPIO.setup(RBP, GPIO.OUT)  # Right ~
GPIO.setup(LBD, GPIO.OUT)  # Left Behind direction
GPIO.setup(LBP, GPIO.OUT)  # Left ~

# PWM Create
LFPPIN = GPIO.PWM(LFP, 360)  # Freq = 360
RFPPIN = GPIO.PWM(RFP, 360)
LBPPIN = GPIO.PWM(LBP, 360)
RBPPIN = GPIO.PWM(RBP, 360)

# PWM start
LFPPIN.start(0)
RFPPIN.start(0)
LBPPIN.start(0)
RBPPIN.start(0)


# Control speed
def LFspeed(speed):
    LFPPIN.ChangeDutyCycle(speed)


def RFspeed(speed):
    RFPPIN.ChangeDutyCycle(speed)


def LBspeed(speed):
    LBPPIN.ChangeDutyCycle(speed)


def RBspeed(speed):
    RBPPIN.ChangeDutyCycle(speed)


# LeftF direction
def LFDir1():
    GPIO.output(LFD, GPIO.HIGH)


def LFDir2():
    GPIO.output(LFD, GPIO.LOW)


# RightF direction
def RFDir1():
    GPIO.output(RFD, GPIO.HIGH)


def RFDir2():
    GPIO.output(RFD, GPIO.LOW)


# LeftB direction
def LBDir1():
    GPIO.output(LBD, GPIO.HIGH)


def LBDir2():
    GPIO.output(LBD, GPIO.LOW)


# RightB direction
def RBDir1():
    GPIO.output(RBD, GPIO.HIGH)


def RBDir2():
    GPIO.output(RBD, GPIO.LOW)


# change direction
def go_forward(speed):
    LFDir2()
    LFspeed(speed)
    RFDir1()
    RFspeed(speed)
    LBDir2()
    LBspeed(speed)
    RBDir1()
    RBspeed(speed)

def turn_to_angle(angle):
    pass

def Goleft(speed):
    LFDir1()
    LFspeed(speed)
    RFDir1()
    RFspeed(speed)
    LBDir2()
    LBspeed(speed)
    RBDir2()
    RBspeed(speed)


def Goleftup(speed):
    LFspeed(0)
    RFDir1()
    RFspeed(speed)
    LBDir2()
    LBspeed(speed)
    RBspeed(0)


def Gorightup(speed):
    LFDir2()
    LFspeed(speed)
    RFspeed(0)
    LBspeed(0)
    RBDir1()
    RBspeed(speed)


def Goleftdown(speed):
    LFDir1()
    LFspeed(speed)
    RFspeed(0)
    LBspeed(0)
    RBDir2()
    RBspeed(speed)


def Gorightdown(speed):
    LFspeed(0)
    RFDir2()
    RFspeed(speed)
    LBDir1()
    LBspeed(speed)
    RBspeed(0)


def Turnleft(speed):
    LFDir1()
    LFspeed(speed)
    RFDir1()
    RFspeed(speed)
    LBDir1()
    LBspeed(speed)
    RBDir1()
    RBspeed(speed)


def Goright(speed):
    LFDir2()
    LFspeed(speed)
    RFDir2()
    RFspeed(speed)
    LBDir1()
    LBspeed(speed)
    RBDir1()
    RBspeed(speed)


def Turnright(speed):
    LFDir2()
    LFspeed(speed)
    RFDir2()
    RFspeed(speed)
    LBDir2()
    LBspeed(speed)
    RBDir2()
    RBspeed(speed)


def Gobackward(speed):
    LFDir1()
    LFspeed(speed)
    RFDir2()
    RFspeed(speed)
    LBDir1()
    LBspeed(speed)
    RBDir2()
    RBspeed(speed)


def stop():
    LFspeed(0)
    RFspeed(0)
    LBspeed(0)
    RBspeed(0)


# TSET
# speed = int(input('Enter speed:'))
# KEY = ();
# while KEY != 'F':
#     KEY = input('Enter Dirction:')
#     if KEY == 'SPEED':
#         speed = int(input('Enter speed:'))
#     if KEY == 'W':
#         go_forward(speed)
#     elif KEY == 'E':
#         Gorightup(speed)
#     elif KEY == 'D':
#         Goright(speed)
#     elif KEY == 'C':
#         Gorightdown(speed)
#     elif KEY == 'X':
#         Gobackward(speed)
#     elif KEY == 'Z':
#         Goleftdown(speed)
#     elif KEY == 'A':
#         Goleft(speed)
#     elif KEY == 'Q':
#         Goleftup(speed)
#     elif KEY == 'S':
#         stop()

# PWM stop
LFPPIN.stop(0)
RFPPIN.stop(0)
LBPPIN.stop(0)
RBPPIN.stop(0)

# Clear GPIO
GPIO.cleanup()


