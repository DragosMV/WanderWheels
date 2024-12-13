"""
 Use this file to generate random points to test calculation methods on
 Within the for loop, modify these lines to change the range of coordinates the point is generated within
        x4_ideal = random.uniform(-15, 15)
        y4_ideal = random.uniform(-15, 15)
 Modify or remove Step 4 to increase, decrease or remove the added error to the measurements
 To replace the calculation method, replace the function at Step 5 (make sure the parameters match)
"""
import random
import math
import numpy as np
from scipy.optimize import least_squares

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
  A = np.array([[2*(x2-x1), 2*(y2-y1)],
                [2*(x3-x2), 2*(y3-y2)]])
  b = np.array([(x2**2 - x1**2) + (y2**2 - y1**2) + (d1**2 - d2**2),
                (x3**2 - x2**2) + (y3**2 - y2**2) + (d2**2 - d3**2)])

  # Solve the linear equation to get the coordinates of p4
  p4 = np.linalg.solve(A, b)
  relative_position = np.array(p4)
  distance = np.linalg.norm(relative_position)
  angle_degrees = calculate_angle(p4[0], p4[1])
  return relative_position, distance, angle_degrees

def find_point_least_squares(p1, p2, p3, d1, d2, d3):
    def distance_residuals(p, d1, d2, d3):
        x4, y4 = p  # Coordinates of the unknown point P4
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

generated_points = 10000
error_range = 0.2
p4_range = 20

# x1, y1, x2, y2, x3, y3
modules_coordinates = [(0, 0.25, -0.2, 0, 0.2, 0)]

# Step 1: Generate random coordinates for p4 (x4, y4)
unknown_points = []
for i in range(generated_points):
    unknown_points.append((random.uniform(-p4_range, p4_range), random.uniform(-p4_range, p4_range)))

for x1, y1, x2, y2, x3, y3 in modules_coordinates:
    distance_error_1, distance_error_2 = 0, 0
    angle_error_1, angle_error_2 = 0, 0

    for x4_ideal, y4_ideal in unknown_points:

        # Step 2: Calculate the distance from p4 to each of the three points
        distance_p1_p4_ideal = calculate_distance(x1, y1, x4_ideal, y4_ideal)
        distance_p2_p4_ideal = calculate_distance(x2, y2, x4_ideal, y4_ideal)
        distance_p3_p4_ideal = calculate_distance(x3, y3, x4_ideal, y4_ideal)

        # Step 3: Calculate angle and distance from (0, 0) to p4
        angle_degrees_ideal = calculate_angle(x4_ideal, y4_ideal)
        distance_ideal = calculate_distance(x4_ideal, y4_ideal, 0, 0)

        # Test validation data (angle, distance and coordinates) are now ready and stored in angle_degrees, distance, x4, y4

        # Step 4: Add error of + or - 15cm to measured distances
        distance_p1_p4 = distance_p1_p4_ideal + random.uniform(0, error_range)
        distance_p2_p4 = distance_p2_p4_ideal + random.uniform(0, error_range)
        distance_p3_p4 = distance_p3_p4_ideal + random.uniform(0, error_range)

        # Step 5: Use trilateration method to calculate 4th point
        coordinates_1, distance_1, angle_degrees_1 = find_point((x1, y1), (x2, y2), (x3, y3), distance_p1_p4, distance_p2_p4, distance_p3_p4)
        coordinates_2, distance_2, angle_degrees_2 = find_point_least_squares((x1, y1), (x2, y2), (x3, y3), distance_p1_p4, distance_p2_p4, distance_p3_p4)

        # Step 6: Calculate errors
        distance_error_1 += abs(distance_ideal - distance_1)
        angle_error_1 += abs(angle_degrees_ideal - angle_degrees_1)
        distance_error_2 += abs(distance_ideal - distance_2)
        angle_error_2 += abs(angle_degrees_ideal - angle_degrees_2)

    print("Average error method 1")
    print(distance_error_1/generated_points, angle_error_1/generated_points)
    print("Average error method 2")
    print(distance_error_2/generated_points, angle_error_2/generated_points)