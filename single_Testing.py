"""
 This file is used to test single test-cases inputted manually
 Used for debugging edge cases
 Can change calculation method by modifying Step 5
 Can change error added to measurement by modifying Step 4
 Can change P4 used for test  manually. Useful tool for verifying results: https://www.desmos.com/calculator/8vzh3adrtj
"""

import random
import math
import numpy as np
from scipy.optimize import least_squares
import time

start_time = time.time()


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
  distance = calculate_distance(p4[0], p4[1], 0, 0)
  angle_degrees = calculate_angle(p4[0], p4[1])
  return p4, distance, angle_degrees

def find_point_least_squares(p1, p2, p3, d1, d2, d3):
    def distance_residuals(p, d1, d2, d3):
        x4, y4 = p  # Coordinates of the unknown point P4
        res = [
            (x4 - x1) ** 2 + (y4 - y1) ** 2 - d1 ** 2,  # Distance to P1
            (x4 - x2) ** 2 + (y4 - y2) ** 2 - d2 ** 2,  # Distance to P2
            (x4 - x3) ** 2 + (y4 - y3) ** 2 - d3 ** 2  # Distance to P3
        ]
        return res

    initial_guess = [5, 5]  # Initial guess for P4
    # Use least_squares to minimize the residuals and find the coordinates of P4
    result = least_squares(distance_residuals, initial_guess, args=(d1, d2, d3))
    # Extract the solution (coordinates of P4)
    x4_new, y4_new = result.x
    distance = calculate_distance(x4_new, y4_new, 0, 0)
    angle_degrees = calculate_angle(x4_new, y4_new)
    return result.x, distance, angle_degrees


x1, y1 = 0, 0.25
x2, y2 = -0.2, 0
x3, y3 = 0.2, 0

# x4_ideal = random.uniform(-15, 15)
# y4_ideal = random.uniform(-15, 15)

x4_ideal = -25
y4_ideal = -25

# Step 2: Calculate the distance from p4 to each of the three points
distance_p1_p4_ideal = calculate_distance(x1, y1, x4_ideal, y4_ideal)
distance_p2_p4_ideal = calculate_distance(x2, y2, x4_ideal, y4_ideal)
distance_p3_p4_ideal = calculate_distance(x3, y3, x4_ideal, y4_ideal)

# Step 3: Calculate angle and distance from (0, 0) to p4
angle_degrees_ideal = calculate_angle(x4_ideal, y4_ideal)
distance_ideal = calculate_distance(x4_ideal, y4_ideal, 0, 0)

# Test validation data (angle, distance and coordinates) are now ready and stored in angle_degrees, distance, x4, y4

# Step 4: Add error of + or - 15cm to measured distances
distance_p1_p4 = distance_p1_p4_ideal + random.uniform(0, 0.15)
distance_p2_p4 = distance_p2_p4_ideal + random.uniform(0, 0.15)
distance_p3_p4 = distance_p3_p4_ideal + random.uniform(0, 0.15)

# Step 5: Use trilateration method to calculate 4th point
coordinates, distance, angle_degrees = find_point_least_squares((x1, y1), (x2, y2), (x3, y3), distance_p1_p4, distance_p2_p4,
                                                  distance_p3_p4)

# Step 6: Compare results
x4, y4 = coordinates
print(abs(angle_degrees - angle_degrees_ideal))
print(abs(distance_ideal - distance))
print("--- %s seconds ---" % (time.time() - start_time))