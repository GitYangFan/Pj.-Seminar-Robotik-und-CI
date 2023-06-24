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
import time


# let the jetbot stop
def stop(jetbot_motor):
    jetbot_motor.set_speed(0, 0)


# let jetbot circle around the object clockwise in front of it
def turn_clockwise(jetbot_motor):
    jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to left
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.2, 0.1)  # turn one circle clockwise
    time.sleep(5.5)
    jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to middle
    time.sleep(1)
    stop(jetbot_motor)


# let jetbot circle around the object counterclockwise in front of it
def turn_counterclockwise(jetbot_motor):
    jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to right
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.1, 0.2)  # turn one circle counterclockwise
    time.sleep(5.5)
    jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to middle
    time.sleep(1)
    stop(jetbot_motor)


# let the jetbot simple go ahead without controller
def ahead_distance(jetbot_motor, distance):
    velocity = 0.1  # unit: m/s
    duration = distance / velocity
    jetbot_motor.set_speed(0.1, 0.1)
    time.sleep(duration)
    stop(jetbot_motor)


# let the jetbot turn to a desired angle
def turn_to_direction(jetbot_motor, direction):
    # direction: desired gradient angle
    position, orientation = apriltag.get_apriltag()
    print('position:', position, 'orientation:', orientation[2] / np.pi * 180)
    difference_cache = (direction - orientation[2])
    difference = ((2 * np.pi) - difference_cache) % (2 * np.pi)
    if difference < np.pi:
        duration = 4 * difference / (2 * np.pi)  # rotate one circle need 4 second
        # turn to the desired direction
        jetbot_motor.set_speed(0.1, -0.1)  # rotate clockwise
        time.sleep(duration)
        stop(jetbot_motor)
    else:
        duration = 4 * (2 * np.pi - difference) / (2 * np.pi)  # rotate one circle need 4 second
        jetbot_motor.set_speed(-0.1, 0.1)  # rotate counterclockwise
        time.sleep(duration)
        stop(jetbot_motor)


# let the jetbot to avoid the obstacle in front of it
def avoid_obstacle(jetbot_motor):
    # avoid the obstacle in front of jetbot
    jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to left
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.2, 0.1)  # turn half circle clockwise to avoid the obstacle
    time.sleep(2.75)
    jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to middle
    time.sleep(1)
    stop(jetbot_motor)


# calculate the distance between jetbot and the desired line
def distance_point_line(line_start, line_end, point):
    numerator = abs(
        (line_end[1] - line_start[1]) * point[0] - (line_end[0] - line_start[0]) * point[1] + line_end[0] * line_start[
            1] - line_end[1] * line_start[0])
    denominator = math.sqrt((line_end[1] - line_start[1]) ** 2 + (line_end[0] - line_start[0]) ** 2)
    distance = numerator / denominator
    return distance


# linear motion mode with a PID controller
def linear_motion(jetbot_motor, end):
    position, orientation = apriltag.get_apriltag()
    start = position[0:2]
    # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
    v1 = end - start
    if v1[0] < 0:
        if v1[1] > 0:
            direction = np.arctan(-v1[0] / v1[1])  # first quadrant
        else:
            direction = np.pi - np.arctan(v1[0] / v1[1])  # second quadrant
    else:
        if v1[1] < 0:
            direction = np.pi + np.arctan(-v1[0] / v1[1])  # third quadrant
        else:
            direction = 2 * np.pi - np.arctan(v1[0] / v1[1])  # fourth quadrant
    turn_to_direction(jetbot_motor, direction)  # turn to the forward direction
    # initialization
    k_left = 0
    k_right = 0
    safe_width = 0.2
    time_init = time.time()
    while 1:
        jetbot_motor.set_speed(0.1 + k_left, 0.1 + k_right)  # go ahead!
        rospy.sleep(0.1)
        position, orientation = apriltag.get_apriltag()
        v2 = position[0:2] - start
        # PID controller (currently only P was considered)
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]  # Calculate the cross product
        if cross_product >= 0:  # too left, must go right
            k_left = 0.4 * distance_point_line(start, end, position[0:2])
            k_right = 0
        elif cross_product < 0:  # too right, must go left
            k_right = 0.4 * distance_point_line(start, end, position[0:2])
            k_left = 0

        # print("Position: ", position[0], position[1])
        # stop if distance between jetbot and end point is less than 0.05 m
        if math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2) < 0.03:
            print("Position: ", position[0], position[1])
            stop(jetbot_motor)
            break

        # stop if move for too long (means go far away from end point)
        if time.time() - time_init > 20:
            print("Warning: time out! Stop at:", position)
            stop(jetbot_motor)
            break

        # stop if jetbot is out of the safe boundary
        if position[0] < (0 + safe_width) or position[0] > (1.485 - safe_width) or position[1] < (0 + safe_width) or \
                position[1] > (1.485 - safe_width):
            print("Warning: out of bounds! Stop at: : ", position)
            stop(jetbot_motor)
            break


"""
-------------------- test ----------------------------
"""
if __name__ == '__main__':
    jetbot_motor = MotorControllerWaveshare()

    # test simple movement
    # ahead_distance(jetbot_motor, 0.2)
    # turn_clockwise(jetbot_motor)
    # turn_counterclockwise(jetbot_motor)

    # test advanced movement
    # turn_to_direction(jetbot_motor, np.pi/2)
    # stop(jetbot_motor)

    linear_motion(jetbot_motor, [0.74, 0.74])
