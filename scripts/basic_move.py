#!/usr/bin/env python
import math
import os
import select
import sys
import rospy
import time
import numpy as np
import apriltag
from motors_waveshare import MotorControllerWaveshare


def stop(jetbot_motor):
    jetbot_motor.set_speed(0, 0)


def turn_clockwise(jetbot_motor):
    jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to left
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.2, 0.1)  # turn one circle clockwise
    time.sleep(5.5)
    stop(jetbot_motor)


def turn_counterclockwise(jetbot_motor):
    jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to right
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.1, 0.2)  # turn one circle counterclockwise
    time.sleep(5.5)
    stop(jetbot_motor)

def ahead_distance(jetbot_motor, distance):
    velocity = 0.1  # unit: m/s
    duration = distance / velocity
    jetbot_motor.set_speed(0.1, 0.1)
    time.sleep(duration)
    stop(jetbot_motor)

def turn_to_direction(direction):
    # direction: desired gradient angle
    position, orientation = apriltag.get_apriltag()
    difference = direction - orientation[2]
    duration = 4 * abs(difference) / (2 * np.pi)  # rotate one circle need 4 second
    if difference >= 0:
        jetbot_motor.set_speed(-0.1, 0.1)   # rotate clockwise
        time.sleep(duration)
    elif difference < 0:
        jetbot_motor.set_speed(0.1, -0.1)   # rotate counterclockwise
        time.sleep(duration)

def distance_point_line(line_start, line_end, point):
    numerator = abs((line_end[1] - line_start[1]) * point[0] - (line_end[0] - line_start[0]) * point[1] + line_end[0] * line_start[1] - line_end[1] * line_start[0])
    denominator = math.sqrt((line_end[1] - line_start[1]) ** 2 + (line_end[0] - line_start[0]) ** 2)
    distance = numerator / denominator
    return distance

def linear_motion(jetbot_motor, end):
    position, orientation = apriltag.get_apriltag()
    start = position[0:1]
    # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
    v1 = end - start
    direction = np.arctan(v1[0]/v1[1])
    turn_to_direction(direction)    # turn in the forward direction
    k_left = 0
    k_right = 0
    while not rospy.is_shutdown:
        jetbot_motor.set_speed(0.1 + k_left, 0.1 + k_right)  # go ahead!
        rospy.sleep(0.1)
        position, orientation = apriltag.get_apriltag()
        v2 = position[0:1] - start
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]   # Calculate the cross product
        if cross_product >= 0:   # too left, must go right
            k_left = 0.1 * distance_point_line(start, end, position[0:1])
            k_right = 0
        elif cross_product < 0:   # too right, must go left
            k_right = 0.1 * distance_point_line(start, end, position[0:1])
            k_left = 0

        if math.sqrt((position[0] - end[0])**2 + (position[1] - end[1])**2)<0.05:
        # stop if distance between jetbot and end point is less than 0.05 m
            stop(jetbot_motor)
            break


"""
-------------------- test ----------------------------
"""
if __name__ == '__main__':
    jetbot_motor = MotorControllerWaveshare()

    # test simple movement
    ahead_distance(jetbot_motor, 0.2)
    turn_clockwise(jetbot_motor)
    turn_counterclockwise(jetbot_motor)

    # test advanced movement
    turn_to_direction(np.pi/2)

    # linear_motion(jetbot_motor, [0.74, 0.74])
