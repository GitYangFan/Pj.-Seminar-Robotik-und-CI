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
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare

    Returns
    ------------
    """
    jetbot_motor.set_speed(0, 0)
    rospy.sleep(0.5)


# let jetbot circle around the object clockwise with the specific angle in front of it
def turn_clockwise(jetbot_motor, angle):  # add angel (unit: radian)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    angle: The desired angle you want the jetbot turn around the object: (unit: radian)

    Returns
    ------------
    """
    print('turn_clockwise...')
    jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to left
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.2, 0.1)  # turn one circle clockwise
    duration = 5.5 * (angle / (2 * np.pi))
    time.sleep(duration)
    # jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to middle
    # time.sleep(1)
    stop(jetbot_motor)
    rospy.sleep(0.5)


# let jetbot circle around the object counterclockwise with the specific angle in front of it
def turn_counterclockwise(jetbot_motor, angle):  # add angel (unit: radian)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    angle: The desired angle you want the jetbot turn around the object: (unit: radian)

    Returns
    ------------
    """
    print('turn_counterclockwise...')
    jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to right
    time.sleep(1)
    stop(jetbot_motor)
    jetbot_motor.set_speed(0.1, 0.2)  # turn one circle counterclockwise
    duration = 5.5 * (angle / (2 * np.pi))
    time.sleep(duration)
    # jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to middle
    # time.sleep(1)
    stop(jetbot_motor)
    rospy.sleep(0.5)


# let the jetbot simple go backwards without controller
def backwards_distance(jetbot_motor, distance):  # distance (unit: m)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    distance: The distance you want the Jetbot go backwards: (unit: m)

    Returns
    ------------
    """
    print('backwards_distance...')
    velocity = 0.1  # wheel velocity (unit: m/s)
    duration = distance / velocity
    jetbot_motor.set_speed(-velocity, -velocity)  # go backwards!
    time.sleep(duration)
    stop(jetbot_motor)
    rospy.sleep(0.5)


# let the jetbot turn to a desired angle
def turn_to_direction(jetbot_motor, direction):  # add apriltag (not for now)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    direction: The desired direction you want the Jetbot turn to : (unit: radian) (y-axis is 0, clockwise [0~2*pi])

    Returns
    ------------
    """
    # direction: desired gradient angle
    print('turn_to_direction...')
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
    rospy.sleep(0.5)


# let the jetbot turn to a desired angle with cube
def turn_to_direction_with_cube(jetbot_motor, direction):
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    direction: The desired direction you want the Jetbot turn to : (unit: radian) (y-axis is 0, clockwise [0~2*pi])

    Returns
    ------------
    """
    # direction: desired gradient angle
    print('turn_to_direction_with_cube...')
    position, orientation = apriltag.get_apriltag()
    print('position:', position, 'orientation:', orientation[2] / np.pi * 180)
    difference_cache = (direction - orientation[2])
    difference = ((2 * np.pi) - difference_cache) % (2 * np.pi)
    time_init = time.time()
    if difference < np.pi:
        duration = 4 * difference / (2 * np.pi)  # rotate one circle need 4 second
        # turn to the desired direction
        jetbot_motor.set_speed(0.165, 0)  # rotate clockwise
        time.sleep(duration)
        stop(jetbot_motor)
    else:
        duration = 4 * (2 * np.pi - difference) / (2 * np.pi)  # rotate one circle need 4 second
        jetbot_motor.set_speed(0, 0.165)  # rotate counterclockwise
        time.sleep(duration)
        stop(jetbot_motor)
    # duration = time.time() - time_init
    # print('duration:', duration)
    rospy.sleep(0.5)


# let the jetbot turn to a desired angle
def turn_to_direction_advanced(jetbot_motor, direction):  # add apriltag to improve the accuracy
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    direction: The desired direction you want the Jetbot turn to : (unit: radian) (y-axis is 0, clockwise [0~2*pi])

    Returns
    ------------
    """
    print('turn_to_direction_advanced...')
    rate = rospy.Rate(50)
    # direction: desired gradient angle (from y-axis counterclockwise)
    position, orientation = apriltag.get_apriltag()
    print('position:', position, 'orientation:', orientation[2] / np.pi * 180)
    difference_cache = (direction - orientation[2])
    difference = ((2 * np.pi) - difference_cache) % (2 * np.pi)
    if difference < np.pi:
        jetbot_motor.set_speed(0.08, 0)  # rotate clockwise
        time_init = time.time()
        while 1:
            position, orientation = apriltag.get_apriltag()
            difference_cache = (direction - orientation[2])
            difference_current = ((2 * np.pi) - difference_cache) % (
                    2 * np.pi)  # current difference compared to desired direction
            print('difference_current:', difference_current)
            # turn to the desired direction
            # rate.sleep()
            if difference_current <= 0.087 * 2:  # Angle error less than 5 degrees (=0.087 radian) is acceptable
                stop(jetbot_motor)
                break
            if time.time() - time_init > 4:
                print("Warning: time out! Try turn_to_direction instead...")
                turn_to_direction(jetbot_motor, direction)
                break

    else:
        jetbot_motor.set_speed(0, 0.08)  # rotate counterclockwise
        time_init = time.time()
        while 1:
            position, orientation = apriltag.get_apriltag()
            difference_cache = (direction - orientation[2])
            difference_current = ((2 * np.pi) - difference_cache) % (
                    2 * np.pi)  # current difference compared to desired direction
            print('difference_current:', difference_current)
            # turn to the desired direction
            # rate.sleep()
            if difference_current <= 0.087 * 2:  # Angle error less than 5 degrees (=0.087 radian) is acceptable
                stop(jetbot_motor)
                break
            if time.time() - time_init > 4:
                print("Warning: time out! Try turn_to_direction instead...")
                turn_to_direction(jetbot_motor, direction)
                break
    rospy.sleep(0.5)


# let the jetbot to avoid the obstacle in front of it
def avoid_obstacle(jetbot_motor, side):  # choose the side you want to go (left: side=0 / right: side=1)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    side: choose the side you want to go (left: side=0 / right: side=1)

    Returns
    ------------
    """
    print('avoid_obstacle...')
    # avoid the obstacle in front of jetbot
    if side == 0:  # go from the left side
        jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to left
        time.sleep(1)
        stop(jetbot_motor)
        jetbot_motor.set_speed(0.2, 0.1)  # turn half circle clockwise to avoid the obstacle
        time.sleep(2.75)
        jetbot_motor.set_speed(-0.1, 0.1)  # Rotate pi/2 in place to middle
        time.sleep(1)
        stop(jetbot_motor)
    else:  # go from the right side
        jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to right
        time.sleep(1)
        stop(jetbot_motor)
        jetbot_motor.set_speed(0.1, 0.2)  # turn half circle counterclockwise to avoid the obstacle
        time.sleep(2.75)
        jetbot_motor.set_speed(0.1, -0.1)  # Rotate pi/2 in place to middle
        time.sleep(1)
        stop(jetbot_motor)
    rospy.sleep(0.5)


# let the jetbot to avoid the obstacle in front of it with cube
def avoid_obstacle_with_cube(jetbot_motor, side):  # choose the side you want to go (left: side=0 / right: side=1)
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    side: choose the side you want to go (left: side=0 / right: side=1)

    Returns
    ------------
    """
    print('avoid_obstacle_with_cube...')
    # avoid the obstacle in front of jetbot
    if side == 0:  # go from the left side
        jetbot_motor.set_speed(0, 0.165)  # Rotate pi/2 in place to left
        time.sleep(1)
        stop(jetbot_motor)
        jetbot_motor.set_speed(0.2, 0.1)  # turn half circle clockwise to avoid the obstacle
        time.sleep(2.75)
        jetbot_motor.set_speed(0, 0.165)  # Rotate pi/2 in place to middle
        time.sleep(1)
        stop(jetbot_motor)
    else:  # go from the right side
        jetbot_motor.set_speed(0.165, 0)  # Rotate pi/2 in place to right
        time.sleep(1)
        stop(jetbot_motor)
        jetbot_motor.set_speed(0.1, 0.2)  # turn half circle counterclockwise to avoid the obstacle
        time.sleep(2.75)
        jetbot_motor.set_speed(0.165, 0)  # Rotate pi/2 in place to middle
        time.sleep(1)
        stop(jetbot_motor)
    rospy.sleep(0.5)


# calculate the distance between jetbot and the desired line
def distance_point_line(line_start, line_end, point):
    """
    Parameters
    ------------
    line_start: The start point of the line
    line_end: The end point of the line
    point: The third point

    Returns
    ------------
    """
    numerator = abs(
        (line_end[1] - line_start[1]) * point[0] - (line_end[0] - line_start[0]) * point[1] + line_end[0] * line_start[
            1] - line_end[1] * line_start[0])
    denominator = math.sqrt((line_end[1] - line_start[1]) ** 2 + (line_end[0] - line_start[0]) ** 2)
    distance = numerator / denominator
    return distance


# linear motion mode with a PID controller
def linear_motion(jetbot_motor, end):
    """
    Parameters
    ------------
    jetbot_motor: the Motor class from motors_waveshare
    end: the desired destination point: [x, y] (unit: m)

    Returns
    ------------
    flag: If move to the destination successfully: flag = True; If out of control: flag = False  (Bool)
    """
    print('linear_motion...')
    print('Going to:', end)
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
    print('turn to direction:', direction)
    rospy.sleep(0.5)
    turn_to_direction(jetbot_motor, direction)  # call this function again to compensate the error
    # print('direction_aft', direction)
    rospy.sleep(0.5)

    # in the case of too short distance
    distance = math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2)
    if distance < 0.1:
        print('distance too short! simple forwards mode...')
        duration = distance / 0.1
        jetbot_motor.set_speed(0.1, 0.1)
        time.sleep(duration)
        stop(jetbot_motor)
        flag = True
        return flag

    # initialization
    k_left = 0
    k_right = 0
    safe_width = 0.3
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
        # stop if distance between jetbot and end point is less than 0.03 m
        if math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2) < 0.05:
            print("Arrive at destination: ", position)
            flag = True  # return True because jetbot get to the destination
            stop(jetbot_motor)
            break

        # stop if move for too long (means go far away from end point)
        if time.time() - time_init > 20:
            print("Warning: time out! Stop at:", position)
            flag = False  # return False because time out
            stop(jetbot_motor)
            break

        # stop if jetbot is out of the safe boundary
        if position[0] < (0 + safe_width) or position[0] > (1.485 - safe_width) or position[1] < (0 + safe_width) or \
                position[1] > (1.485 - safe_width):
            print("Warning: out of bounds! Stop at: : ", position)
            flag = False  # return False because out of bounds
            stop(jetbot_motor)
            break

    rospy.sleep(0.5)
    return flag  # return the flag (Bool), move successfully: flag = True; out of control: flag = False


# linear motion mode with a PID controller and a desired time
def linear_motion_with_desired_time(jetbot_motor, end, t):
    print('linear_motion_with_desired_time...')
    print('Going to:', end, 'with desired time:', t)
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

    turn_to_direction(jetbot_motor, direction)
    print('turn to direction:', direction)
    rospy.sleep(0.5)
    turn_to_direction(jetbot_motor, direction)  # turn to the forward direction
    # print('direction_aft', direction)
    rospy.sleep(0.5)

    # in the case of too short distance
    distance = math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2)
    if distance < 0.1:
        print('distance too short! simple forwards mode...')
        duration = distance / 0.1
        jetbot_motor.set_speed(0.1, 0.1)
        time.sleep(duration)
        stop(jetbot_motor)
        flag = True
        return flag

    # initialization
    k_left = 0
    k_right = 0
    safe_width = 0.1
    MAX_TIME = 10
    if t > MAX_TIME:
        t = MAX_TIME
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
        # stop if distance between jetbot and end point is less than 0.03 m
        if math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2) < 0.05:
            print("Arrive at destination: ", position)
            flag = True  # return True because jetbot get to the destination
            stop(jetbot_motor)
            break

        # stop if move for desired time, but not too long
        if time.time() - time_init > t:
            if t == MAX_TIME:
                print("Warning: time out! Stop at:", position)
                flag = False  # return False because time out
                stop(jetbot_motor)
                break
            else:
                print("Move for a desired time! Stop at:", position)
                flag = True
                stop(jetbot_motor)
                break

        # stop if jetbot is out of the safe boundary
        if position[0] < (0 + safe_width) or position[0] > (1.485 - safe_width) or position[1] < (0 + safe_width) or \
                position[1] > (1.485 - safe_width):
            print("Warning: out of bounds! Stop at: : ", position)
            flag = False  # return False because out of bounds
            stop(jetbot_motor)
            break

    rospy.sleep(0.5)
    return flag  # return the flag (Bool), move successfully: flag = True; out of control: flag = False


# linear motion mode with a PID controller and a desired time
def linear_motion_with_desired_time_with_cube(jetbot_motor, end, t):
    print('linear_motion_with_desired_time...')
    print('Going to:', end, 'with desired time:', t)
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

    turn_to_direction_with_cube(jetbot_motor, direction)
    print('turn to direction:', direction)
    rospy.sleep(0.5)
    # turn_to_direction(jetbot_motor, direction)  # turn to the forward direction
    # print('direction_aft', direction)
    # rospy.sleep(0.5)

    # in the case of too short distance
    distance = math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2)
    if distance < 0.1:
        print('distance too short! simple forwards mode...')
        duration = distance / 0.1
        jetbot_motor.set_speed(0.1, 0.1)
        time.sleep(duration)
        stop(jetbot_motor)
        flag = True
        return flag

    # initialization
    k_left = 0
    k_right = 0
    safe_width = 0.1
    MAX_TIME = 10
    if t > MAX_TIME:
        t = MAX_TIME
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
        # stop if distance between jetbot and end point is less than 0.03 m
        if math.sqrt((position[0] - end[0]) ** 2 + (position[1] - end[1]) ** 2) < 0.05:
            print("Arrive at destination: ", position)
            flag = True  # return True because jetbot get to the destination
            stop(jetbot_motor)
            break

        # stop if move for desired time, but not too long
        if time.time() - time_init > t:
            if t == MAX_TIME:
                print("Warning: time out! Stop at:", position)
                flag = False  # return False because time out
                stop(jetbot_motor)
                break
            else:
                print("Move for a desired time! Stop at:", position)
                flag = True
                stop(jetbot_motor)
                break

        # stop if jetbot is out of the safe boundary
        if position[0] < (0 + safe_width) or position[0] > (1.485 - safe_width) or position[1] < (0 + safe_width) or \
                position[1] > (1.485 - safe_width):
            print("Warning: out of bounds! Stop at: : ", position)
            flag = False  # return False because out of bounds
            stop(jetbot_motor)
            break

    rospy.sleep(0.5)
    return flag  # return the flag (Bool), move successfully: flag = True; out of control: flag = False


"""
-------------------- test ----------------------------
"""
"""
if __name__ == '__main__':
    jetbot_motor = MotorControllerWaveshare()

    # test simple movement

    print('Motion mode: backwards_distance')
    # backwards_distance(jetbot_motor, 0.2)

    print('Motion mode: turn_clockwise')
    # turn_clockwise(jetbot_motor, np.pi)

    print('Motion mode: turn_counterclockwise')
    # turn_counterclockwise(jetbot_motor, 2 * np.pi)

    print('Motion mode: avoid_obstacle')
    # avoid_obstacle(jetbot_motor, side=1)

    print('Motion mode: turn_to_direction')
    # turn_to_direction(jetbot_motor, np.pi)
    # turn_to_direction_advanced(jetbot_motor, np.pi)
    turn_to_direction_with_cube(jetbot_motor, np.pi / 2)

    # test advanced movement

    print('Motion mode: turn_to_direction_advanced')
    # turn_to_direction_advanced(jetbot_motor, np.pi/2)

    print('Motion mode: linear_motion')
    # flag = linear_motion(jetbot_motor, [0.6, 0.3])
    # print('flag is:', flag)

    print('Motion mode: linear_motion_with_desired_time')
    # flag = linear_motion_with_desired_time(jetbot_motor, [0.6, 0.3], 3)
    # print('flag is:',flag)

    stop(jetbot_motor)
    
"""
