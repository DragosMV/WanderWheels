import math
import numpy as np
from scipy.optimize import least_squares

def get_distance_and_angle(d1, d2, d3, d4):

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

    def find_point(p1, p2, p3, p4, d1, d2, d3, d4):
        """
        Given 4 points and each distance between them to a 5th point,
        Finds the coordinates, distance and angle from (0,0) to that 5th point
        """
        x1, y1, x2, y2, x3, y3, x4, y4 = p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]
        # Create a matrix A and vector b for the linear equation Ax = b
        A = np.array([
            [2 * (x2 - x1), 2 * (y2 - y1)],
            [2 * (x3 - x2), 2 * (y3 - y2)],
            [2 * (x4 - x3), 2 * (y4 - y3)]
        ])
        b = np.array([
            (x2 ** 2 - x1 ** 2) + (y2 ** 2 - y1 ** 2) + (d1 ** 2 - d2 ** 2),
            (x3 ** 2 - x2 ** 2) + (y3 ** 2 - y2 ** 2) + (d2 ** 2 - d3 ** 2),
            (x4 ** 2 - x3 ** 2) + (y4 ** 2 - y3 ** 2) + (d3 ** 2 - d4 ** 2)
        ])

        # Solve the least-squares equation to get the coordinates of p5
        p5, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        relative_position = np.array(p5)
        distance = np.linalg.norm(relative_position)
        angle_degrees = calculate_angle(p5[0], p5[1])
        return relative_position, distance, angle_degrees

    def find_point_least_squares(p1, p2, p3, p4, d1, d2, d3, d4):
        def distance_residuals(p, d1, d2, d3, d4):
            x5, y5 = p  # Coordinates of the unknown point P5
            x1, y1, x2, y2, x3, y3, x4, y4 = (
                p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]
            )
            res = [
                np.sqrt((x5 - x1) ** 2 + (y5 - y1) ** 2) - d1,  # Distance to P1
                np.sqrt((x5 - x2) ** 2 + (y5 - y2) ** 2) - d2,  # Distance to P2
                np.sqrt((x5 - x3) ** 2 + (y5 - y3) ** 2) - d3,  # Distance to P3
                np.sqrt((x5 - x4) ** 2 + (y5 - y4) ** 2) - d4  # Distance to P4
            ]
            return res

        initial_guess = [0, 0]  # Initial guess for P5

        # Use least_squares to minimize the residuals and find the coordinates of P5
        result = least_squares(distance_residuals, initial_guess, args=(d1, d2, d3, d4))
        # Extract the solution (coordinates of P5)
        x5_new, y5_new = result.x
        distance = calculate_distance(x5_new, y5_new, 0, 0)
        angle_degrees = calculate_angle(x5_new, y5_new)
        return result.x, distance, angle_degrees

    # Example points for the anchors
    p1, p2, p3, p4 = (0, 0.25), (-0.2, 0), (0.2, 0), (0, -0.25)

    coordinates_1, distance_1, angle_degrees_1 = find_point_least_squares(
        p1, p2, p3, p4, d1, d2, d3, d4
    )
    coordinates_2, distance_2, angle_degrees_2 = find_point(
        p1, p2, p3, p4, d1, d2, d3, d4
    )

    # Use the results of the first function by default
    result_distance, result_angle = distance_1, angle_degrees_1

    # Validation checks
    if (abs(distance_1 - distance_2) > 10) or (abs(angle_degrees_1 - angle_degrees_2) > 30):
        print("Aborting, please retry calculation")
        return False

    return result_distance, result_angle

# Quick test with four distances https://www.desmos.com/calculator/z9umejqplh
print(get_distance_and_angle(13.96, 14.28, 14.00, 14.32))