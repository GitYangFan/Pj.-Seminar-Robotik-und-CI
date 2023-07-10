from object_localization import get_object_position
from object_localization import find_object
from object_localization import get_objects
from basic_move import *
import rospy
import numpy as np
from basic_move import turn_to_direction
import math
from apriltag import get_apriltag
from motors_waveshare import MotorControllerWaveshare
import matplotlib.pyplot as plt
import time



a=[0]*10
a[0]=[1.3,0.2]
a[1]=[1.3,0.5]
a[2]=[0.2,0.5]
a[3]=[0.2,0.8]
a[4]=[1.3,0.8]
a[5]=[1.3,1.1]
a[6]=[0.2,1.1]
a[7]=[0.2,1.3]
a[8]=[1.3,1.3]
a[9]=[0.1,0.1]


def find_nearest_cube(object):
    """
    Return the nearest cube in one detection

    Args:
        cubes (list of object): the cubes, which are found in one detection

    Returns:
        nearest_cube (ObjecT): the nearest cube in one detection
    """
    min_distance = object[0].distance
    min_index = 0
    for i in range(len(object)):
        if object[i].distance < min_distance:
            min_distance = object[i].distance
            min_index = i

    return object[min_index]

def approach_obstacle(jetbot_motor,obstacle_position):
    """
    Approach the obstacle for avoiding

    Args:
        obstacle_position ([float, float]): the position of the obstacle need to avoid
    """
    DISTANCE_TO_OBSTACLE = 0.10
    position, _ = get_apriltag()
    vec = [obstacle_position[0] - position[0], obstacle_position[1] - position[1]]
    magnitude = math.sqrt(vec[0]**2 + vec[1]**2)
    uVec = [vec[0] / magnitude, vec[1] / magnitude]
    p_x = (-DISTANCE_TO_OBSTACLE * uVec[0]) + obstacle_position[0]
    p_y = (-DISTANCE_TO_OBSTACLE * uVec[1]) + obstacle_position[1]
    p = [p_x, p_y]
    linear_motion(jetbot_motor, p)

a=[0]*10
a[0]=[1.3,0.2]
a[1]=[1.3,0.5]
a[2]=[0.2,0.5]
a[3]=[0.2,0.8]
a[4]=[1.3,0.8]
a[5]=[1.3,1.1]
a[6]=[0.2,1.1]
a[7]=[0.2,1.3]
a[8]=[1.3,1.3]
a[9]=[0.1,0.1]

jetbot_motor = MotorControllerWaveshare()
turn_to_direction(jetbot_motor, 3*np.pi/2)
position, _ = get_apriltag()
objects= get_objects()
if objects!=[]:
    nearest_object= find_nearest_cube(objects)
    object_distance =  nearest_object.distance

for i in range(10):
    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
    while distance_to_destination > 0.08 :
        position, _ = get_apriltag()
        if position[0] > 1.35 and position[0] < 0.1:
            position, _ = get_apriltag()
        else:
            objects = get_objects()
            if objects !=[]:
                nearest_object = find_nearest_cube(objects)
                object_distance = nearest_object.distance
                object_position= nearest_object.position
                if object_distance < 0.2:
                    approach_obstacle(jetbot_motor,object_position)
                    direction = get_direction(object_position[0],object_position[1])
                    turn_to_direction(jetbot_motor, direction)
                    position, _ = get_apriltag()
                    if position[1] < 1.3 and position[0] > 0.3:
                        side = 0  # left
                        avoid_obstacle(jetbot_motor, side)
                    else:
                        side = 1  # right
                        avoid_obstacle(jetbot_motor, side)
            else:
                linear_motion_with_desired_time(jetbot_motor, a[i],1.5)
                distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)

