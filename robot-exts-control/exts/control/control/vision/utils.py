import numpy as np
import cv2

def find_bottom_point(points):
    """
    Find the bottom point from a list of points
    """
    bottom_point = points[0]
    for point in points:
        if point[1] > bottom_point[1]:
            bottom_point = point
    return bottom_point

def find_left_point(points):
    """
    Find the left point from a list of points
    """
    left_point = points[0]
    for point in points:
        if point[0] < left_point[0]:
            left_point = point
    return left_point

def get_projection(point, direction, z):
    """
    Get projection
    """
    t = (z - point[2]) / direction[2]
    x = point[0] + direction[0] * t
    y = point[1] + direction[1] * t
    return (x, y, z)