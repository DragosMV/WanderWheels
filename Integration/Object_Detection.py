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

# Define the GPIO pins for the ultrasonic sensor
TRIG_PIN = 4
ECHO_PIN = 17

# GPIO set up
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(RFD, GPIO.OUT)
GPIO.setup(RFP, GPIO.OUT)
GPIO.setup(LFD, GPIO.OUT)
GPIO.setup(LFP, GPIO.OUT)
GPIO.setup(RBD, GPIO.OUT)
GPIO.setup(RBP, GPIO.OUT)
GPIO.setup(LBD, GPIO.OUT)
GPIO.setup(LBP, GPIO.OUT)

# Set up the ultrasonic sensor pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Define the distance threshold in cm (adjust as needed)
REDDISTANCE_THRESHOLD = 60
YELDISTANCE_THRESHOLD = 120

state = 0


# Get distance function
def get_distance():
    # Send a trigger signal
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Wait for the echo response
    pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the distance in centimeters
    pulse_duration = pulse_end - pulse_start
    speed_of_sound = 34300  # Speed of sound in cm/s
    distance = (pulse_duration * speed_of_sound) / 2

    return distance


# PWM Create
LFPPIN = GPIO.PWM(LFP, 360)
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
def Goforward(speed):
    LFDir2()
    LFspeed(speed)
    RFDir1()
    RFspeed(speed)
    LBDir2()
    LBspeed(speed)
    RBDir1()
    RBspeed(speed)


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


#### Red Zone/Stop Motor Function
def Stop():
    print("Distance below red threshold. Stopping motors.")
    LFspeed(0)
    RFspeed(0)
    LBspeed(0)
    RBspeed(0)


state = 1
return state


#### Yellow Zone/Half Speed Function
def yellow():
    print("Distance between yellow and red threshold. Halving motor speed.")
    state = 2
    return state


#### Green/Normal Speed Fucntion
def green():
    print("Distance in green threshold. Normal motor speed.")
    state = 3
    return state


def out_of_range(state):
    if state == 1:
        Stop()
    elif state == 2:
        New_Speed = speed // 2
    yellow()
    elif state == 3:
    New_Speed = speed


green()

else:
print("Distance out of range. No change.")

try:
    speed = int(input('Enter speed:'))
KEY = ();
while True:
    KEY = input('Enter Command:')
    if KEY == 'SPEED':
        speed = int(input('Enter initial speed:'))

    # Get the distance from the ultrasonic sensor
distance = get_distance()
print("Distance:", distance, "cm")

# If the distance is below the threshold of 60cm motors STOP
if distance < REDDISTANCE_THRESHOLD:
    Stop()
    state = Stop()
    print(state)

# If the distance is between 60cm and 120cm, motors halve speed
elif distance < YELDISTANCE_THRESHOLD and distance > REDDISTANCE_THRESHOLD:
    yellow()
    state = yellow()
    print(state)
    # Speed will update to become half
    New_Speed = speed // 2

elif distance > 600:
    out_of_range(state)

else:
    green()
    state = green()
    # Speed stays as normal
    New_Speed = speed

    if KEY == 'W':
        Goforward(New_Speed)
    elif KEY == 'E':
        Gorightup(New_Speed)
    elif KEY == 'D':
    Goright(New_Speed)
    elif KEY == 'C':
    Gorightdown(New_Speed)
    elif KEY == 'X':
    Gobackward(New_Speed)
    elif KEY == 'Z':
    Goleftdown(New_Speed)
    elif KEY == 'A':
    Goleft(New_Speed)
    elif KEY == 'Q':
    Goleftup(New_Speed)
    elif KEY == 'S':
    Stop()

# Add a small delay to avoid excessive readings
time.sleep(0.5)

except KeyboardInterrupt:
Stop()

finally:
# PWM stop
LFPPIN.stop(0)
RFPPIN.stop(0)
LBPPIN.stop(0)
RBPPIN.stop(0)

# Clear GPIO
GPIO.cleanup()

