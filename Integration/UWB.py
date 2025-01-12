# This function takes 3 inputs and returns 2 outputs
# The inputs are the measured distances received from the UWB modules - d1, d2, d3
# The outputs are the distance and angle towards the user
# The function will do some checks to make sure the results are reasonable
# It may return false if the results present problems

import math
import numpy as np
from scipy.optimize import least_squares

def get_distance_and_angle(d1, d2, d3):

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

    coordinates_1, distance_1, angle_degrees_1 = find_point_least_squares((0, 0.25), (-0.2, 0), (0.2, 0), d1,d2, d3)
    coordinates_2, distance_2, angle_degrees_2 = find_point((0, 0.25), (-0.2, 0), (0.2, 0), d1, d2, d3)

    # by default, use the results of the first function
    result_distance, result_angle = distance_1, angle_degrees_1

    # if the difference between the calculated distances is more than 15m or the difference
    # between the angles is more than 30 degrees, something has gone terribly wrong

    if (abs(distance_1-distance_2) > 10) or (abs(angle_degrees_1 - angle_degrees_2) > 30):
        print("Aborting, please retry calculation")
        return False

    return result_distance, result_angle

# quick test with point (10, 10) https://www.desmos.com/calculator/flvxtfehnc

print(get_distance_and_angle(13.96, 14.28, 14.00))
