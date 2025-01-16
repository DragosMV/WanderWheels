import RPi.GPIO as GPIO
import time
from time import sleep

# Define the GPIO pins for the ultrasonic sensor
TRIG_PIN = 4
ECHO_PIN = 17

# GPIO set up
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# Set up the ultrasonic sensor pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Define the distance threshold in cm (adjust as needed)
REDDISTANCE_THRESHOLD = 60

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


#### Red Zone/Stop Motor Function
def red():
    print("Distance below red threshold. Stopping motors.")
    return 1


#### Green/Normal Speed Fucntion
def green():
    print("Distance in green threshold. Normal motor speed.")
    return 3


def out_of_range(state):
    if state == 1:
        red()
    elif state == 3:
        green()
    else:
        print("Distance out of range. No change.")

try:
    while True:
        # Get the distance from the ultrasonic sensor
        distance = get_distance()
        print("Distance:", distance, "cm")

        # If the distance is below the threshold of 60cm motors STOP
        if distance < REDDISTANCE_THRESHOLD:
            state = red()
            print(state)
        elif distance > 600:
            out_of_range(state)
        else:
            state = green()

        # Add a small delay to avoid excessive readings
        sleep(0.5)

except:
    print("Error occurred in object detection")

finally:
    # Clear GPIO
    GPIO.cleanup()
