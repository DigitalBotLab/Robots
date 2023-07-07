import numpy as np
import cv2
from pxr import Gf

BOX_SIZE = [0.071, 0.0965, 0.1198] # in cm

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
    return np.array((x, y, z))

def get_box_transform_from_point(camera_position, bottom_direction, left_direction, affordance_z = 0):
    """
    Get box points
    """
    bottom_point = get_projection(camera_position, bottom_direction, affordance_z)
    left_point = get_projection(camera_position, left_direction, affordance_z)

    distance =  np.linalg.norm(bottom_point - left_point)
    
    closest_value = min(BOX_SIZE,key=lambda x:abs(x-distance))
    print("distance: ", distance, bottom_point, left_point, "\n close to: ", closest_value)

    direction = left_point - bottom_point
    direction = direction / np.linalg.norm(direction)
    direction = Gf.Vec3d(direction[0], direction[1], direction[2])
    print("direction: ", direction)
    
    # determine the box rotation
    if closest_value == BOX_SIZE[0]:  
        direction_r = np.array([direction[1], -direction[0], 0])
        right_point = bottom_point + direction_r * BOX_SIZE[1]
        center_point = (left_point + right_point) / 2
        rotation = Gf.Rotation(Gf.Vec3d(0, -1, 0), direction)
    elif closest_value == BOX_SIZE[1]:
        direction_r = np.array([direction[1], -direction[0], 0])
        right_point = bottom_point + direction_r * BOX_SIZE[0]
        center_point = (left_point + right_point) / 2
        rotation = Gf.Rotation(Gf.Vec3d(-1, 0, 0), direction)
    else:
        
        center_point = (left_point + bottom_point) / 2
        from_direction = Gf.Vec3d([BOX_SIZE[1], -BOX_SIZE[1], 0]).GetNormalized()
        rotation = Gf.Rotation(from_direction, direction)

    return center_point, rotation.GetQuat()

