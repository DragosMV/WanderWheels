## script to set up pi as BLE server for iphone app to connect to. Has characteristics which enable the iphone app to read and write to the charactersitics for trnamission of data
import sys, time, logging, asyncio, threading, struct, math
import numpy as np
import RPi.GPIO as GPIO
from scipy.optimize import least_squares
from typing import Any, Union
from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions
from motor_control import stop, go_forward, turn_to_angle

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

# assign distance and angle GLOBAL variables
angleglobal, distanceglobal = 0, 0
# global distance_angle_tuple
distance_angle_tuple = (0.0, 0.0)
# global distance_angle_packed
distance_angle_packed = bytearray()
state = "STOP"

# set up logger for debug
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(name=__name__)

# setup asynchronous running
trigger: Union[asyncio.Event, threading.Event]
if sys.platform in ["darwin", "win32"]:
    trigger = threading.Event()
else:
    trigger = asyncio.Event()

# Format string matching the packed structure
format_stringtracking = 'i 4x f i 4x f i 4x f'
format_stringbyte = 'B'


# Object detection functions:
def get_distance_ultrasonic():
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
    if state == "STOP":
        red()
    elif state == "RUN":
        green()
    else:
        print("Distance out of range. No change.")


def get_distance_and_angle(d1, d2, d3):
    # This function takes 3 inputs and returns 2 outputs
    # The inputs are the measured distances received from the UWB modules - d1, d2, d3
    # The outputs are the distance and angle towards the user
    # The function will do some checks to make sure the results are reasonable
    # It may return false if the results present problems

    # Function to calculate Euclidean distance between two points
    def calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Function to calculate angle between point and origin
    def calculate_angle(x1, y1):
        angle = np.arctan2(y1, x1)
        angle_deg = angle * 180 / np.pi
        # Adjust angle to be in the range [0, 360]
        if angle_deg < 0:
            angle_deg += 360
        return angle_deg

    def find_point(p1, p2, p3, d1, d2, d3):
        """
        Given 3 points and each distance between them to a 4th point
        Finds the coordinates, distance and angle from (0,0) to that 4th point
        CAN'T BE SOLVED IF POINTS ARE COLLINEAR
        """
        x1, y1, x2, y2, x3, y3 = p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]
        # Create a matrix A and vector b for the linear equation Ax = b
        A = np.array([[2 * (x2 - x1), 2 * (y2 - y1)],
                      [2 * (x3 - x2), 2 * (y3 - y2)]])
        b = np.array([(x2 ** 2 - x1 ** 2) + (y2 ** 2 - y1 ** 2) + (d1 ** 2 - d2 ** 2),
                      (x3 ** 2 - x2 ** 2) + (y3 ** 2 - y2 ** 2) + (d2 ** 2 - d3 ** 2)])

        # Solve the linear equation to get the coordinates of p4
        p4 = np.linalg.solve(A, b)
        relative_position = np.array(p4)
        distance = np.linalg.norm(relative_position)
        angle_degrees = calculate_angle(p4[0], p4[1])
        return relative_position, distance, angle_degrees

    def find_point_least_squares(p1, p2, p3, d1, d2, d3):
        def distance_residuals(p, d1, d2, d3):
            x4, y4 = p  # Coordinates of the unknown point P4
            x1, y1, x2, y2, x3, y3 = p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]
            res = [
                np.sqrt((x4 - x1) ** 2 + (y4 - y1) ** 2) - d1,  # Distance to P1
                np.sqrt((x4 - x2) ** 2 + (y4 - y2) ** 2) - d2,  # Distance to P2
                np.sqrt((x4 - x3) ** 2 + (y4 - y3) ** 2) - d3  # Distance to P3
            ]
            return res

        initial_guess = [0, 0]  # Initial guess for P4

        # Use least_squares to minimize the residuals and find the coordinates of P4
        result = least_squares(distance_residuals, initial_guess, args=(d1, d2, d3))
        # Extract the solution (coordinates of P4)
        x4_new, y4_new = result.x
        distance = calculate_distance(x4_new, y4_new, 0, 0)
        angle_degrees = calculate_angle(x4_new, y4_new)
        return result.x, distance, angle_degrees

    coordinates_1, distance_1, angle_degrees_1 = find_point_least_squares((0, 0.25), (-0.2, 0), (0.2, 0), d1, d2, d3)
    coordinates_2, distance_2, angle_degrees_2 = find_point((0, 0.25), (-0.2, 0), (0.2, 0), d1, d2, d3)

    # by default, use the results of the first function
    result_distance, result_angle = distance_1, angle_degrees_1

    # if the difference between the calculated distances is more than 15m or the difference
    # between the angles is more than 30 degrees, something has gone terribly wrong

    if (abs(distance_1 - distance_2) > 10) or (abs(angle_degrees_1 - angle_degrees_2) > 30):
        print("Aborting, please retry calculation")
        return False

    return result_distance, result_angle


# define read request function
def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    # logger.debug(f"Reading {characteristic.value}")
    if characteristic.uuid == distance_angle_char:
        print(characteristic.uuid)
        print(characteristic.value)
        # update values with new measurements and pack distance and angle data
        server.update_value(my_service_uuid, distance_angle_char)
    return characteristic.value


# define write request function
def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    current_value = value
    characteristic.value = value

    # assign unpacking based on characteristic UUID
    try:
        if characteristic.uuid == tracking_data_char:  # tracking data
            if state == "STOP":
                print("Obstacle detected. Stop")
                return
            # Unpack the received value using the correct format string
            beacon1, distance1, beacon2, distance2, beacon3, distance3 = struct.unpack(format_stringtracking, value)
            # logger.debug(f"Unpacked device ID: {beacon1}:{distance1}, {beacon2}:{distance2}, {beacon3}:{distance3}")

            # get distance and angle reading from trilateration code
            (distance, angle) = get_distance_and_angle(distance1, distance2, distance3)
            global distanceglobal, angleglobal
            distanceglobal = distance
            angleglobal = angle

            # change the distance and angle into a tuple
            distance_angle_tuple = (distance, angle)
            print(distance_angle_tuple)
            # pack the distance and angle
            distance_angle_packed = struct.pack('ff', *distance_angle_tuple)

            # instantiate characteristic to store processed values
            characteristic1 = server.get_characteristic(distance_angle_char)
            # write new values to characteristic
            write_request(characteristic1, distance_angle_packed)

            turn_to_angle(angle)
            # suitcase control
            if distance < 1.00:
                stop()
            elif distance > 10.0:
                # go at high speed
                go_forward(50)
            else:
                # go at low-normal speed
                go_forward(20)

        if characteristic.uuid == distance_angle_char:
            # update values with new measurements and pack distance and angle data
            server.update_value(my_service_uuid, distance_angle_char)

        if characteristic.uuid == follow_switch_char:  # follow flag
            # Unpack the received value using the correct format string
            # print(value)
            followswitchvalue = struct.unpack(format_stringbyte, value)
            # print(followswitchvalue)

    except Exception as e:
        logger.error(f"Error unpacking data: {e}")
        return


# define  main async loop
async def run_ble(loop):
    trigger.clear()

    # Instantiate the server
    my_service_name = "WanderWheels"
    global server
    server = BlessServer(name=my_service_name, loop=loop)

    # assign functions to server
    server.read_request_func = read_request
    server.write_request_func = write_request

    # Add WanderWheels Service
    global my_service_uuid
    my_service_uuid = "A07498CA-AD5B-474E-940D-16F1FBE7E8CD"
    await server.add_new_service(my_service_uuid)

    # characeristic UUID
    global tracking_data_char
    global distance_angle_char
    global follow_switch_char
    tracking_data_char = "51ff12bb-3ed8-46e5-b4f9-d64e2fec021b"
    follow_switch_char = "df0a2a78-c76b-4d4f-95cc-9e92c35c4507"
    distance_angle_char = "02abb070-9204-4beb-93f4-f9c826d57d0d"

    # asssign characteristic properties
    global char_flags
    char_flags = (
            GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.write
            | GATTCharacteristicProperties.indicate
            | GATTCharacteristicProperties.write_without_response
    )

    global permissions
    # assign characteristic permissions
    permissions = GATTAttributePermissions.readable | GATTAttributePermissions.writeable

    # add characteristics to service
    await server.add_new_characteristic(
        my_service_uuid, tracking_data_char, char_flags, bytearray(b'\x00'), permissions
    )
    await server.add_new_characteristic(
        my_service_uuid, distance_angle_char, char_flags, bytearray(b'\x00'), permissions
    )
    await server.add_new_characteristic(
        my_service_uuid, follow_switch_char, char_flags, bytearray(b'\x00'), permissions
    )

    await server.start()
    logger.debug("Advertising")

    if trigger.__module__ == "threading":
        trigger.wait()
    else:
        await trigger.wait()


# Object Detection Thread
def run_object_detection():
    global state

    try:
        while True:
            distance = get_distance_ultrasonic()
            print(f"Distance: {distance:.2f} cm")

            if distance < REDDISTANCE_THRESHOLD:
                state = red()
            elif distance > 600:
                out_of_range(state)
            else:
                state = green()
            time.sleep(0.5)
    except Exception as e:
        print(f"Error in object detection: {e}")
    finally:
        GPIO.cleanup()


# Run Both BLE and Object Detection
def main():
    loop = asyncio.get_event_loop()

    # Start object detection in a separate thread
    object_detection_thread = threading.Thread(target=run_object_detection, daemon=True)
    object_detection_thread.start()

    # Start BLE server
    loop.create_task(run_ble(loop))
    loop.run_forever()


if __name__ == "__main__":
    main()

# loop = asyncio.get_event_loop()
# loop.create_task(run_ble(loop))
# loop.run_forever()


# After trigger event, you can update the value for testing purposes.
# logger.debug("Updating characteristic")

# server.get_characteristic(distance_angle_char)
# server.update_value(my_service_uuid, distance_angle_char)
