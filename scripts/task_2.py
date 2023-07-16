
import numpy as np
from matplotlib import pyplot as plt, patches
from basic_move import linear_motion
from basic_move import *
from motors_waveshare import MotorControllerWaveshare
from detection_results import get_detection
from object_localization  import *
import basic_move

#planned_trajectory to drive
planned_trajectory =[('pointw1', [0.1, 0.1, 0.5, 0.475]), ('kreisw1', [0.5, 0.6, 0.125]), ('kurvew1', [0.375, 0.6]), ('pointw3', [0.375, 0.6, 0.4, 0.975]), ('kreisw3', [0.4, 1.1, 0.125]), ('kurvew3', [0.525, 1.1]), ('pointw6', [0.525, 1.1, 0.975, 1.1]), ('kreisw6', [1.1, 1.1, 0.125]), ('pointw2', [0.975, 1.1, 1.0, 0.825]), ('kreisw2', [1.0, 0.7, 0.125]), ('kurvew2', [1.125, 0.7]), ('pointw5', [1.125, 0.7, 1.2, 0.425]), ('kreisw5', [1.2, 0.3, 0.125]), ('kurvew5', [1.075, 0.3]), ('pointw4', [1.075, 0.3, 0.8, 0.425]), ('kreisw4', [0.8, 0.3, 0.125]), ('kurvew4', [0.675, 0.3]), ('start_point90', [0.675, 0.3, 0.675, 0.1]), ('start_point', [0.675, 0.1, 0.1, 0.1])]
#cubs in order
merged_sorted =[('w1', [0.5, 0.6, 'BLUE']), ('w3', [0.4, 1.1, 'BLUE']), ('w6', [1.1, 1.1, 'RED']), ('w2', [1.0, 0.7, 'BLUE']), ('w5', [1.2, 0.3, 'RED']), ('w4', [0.8, 0.3, 'RED'])]

# save a list in text data
def save_list(liste, dateiname):
    with open(dateiname, 'w') as datei:
        for element in liste:
            datei.write(str(element) + '\n')

# Jetbot's real position
pos_real = []
next_next_ob= 0

jetbot_motor = MotorControllerWaveshare()

# according to the planned trajectory we drive and save our current position
for j in range(len(merged_sorted)):
    next_next_ob = next_next_ob + 1
    last_ob = merged_sorted[-1][0]
    # next next position
    if merged_sorted[j][0] != last_ob:
        jjj = merged_sorted[j + 1]

    for i in range(len(planned_trajectory)):
        # linear motion
        if planned_trajectory[i][0] == "point" + merged_sorted[j][0]:

            # get the Jetbot's current position and the orientation
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            # save the current position in pos_real
            pos_real.append(position[0:2])
            o2=[planned_trajectory[i][1][2], planned_trajectory[i][1][3]]


            # get the required direction
            start = position[0:2]
            # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
            v1 = o2 - start
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
            turn_to_direction(jetbot_motor, direction)
            #detect the ball
            object_name,_, object_position,_ = get_object_position(jetbot_motor)

            # Drive linearly until we reach our destination
            for p in  range(7):
                object_name, _, object_position, _ = get_object_position(jetbot_motor)
                # get the Jetbot's current position and the orientation
                position, orientation = basic_move.get_apriltag(jetbot_motor)
                pos_real.append(position[0:2])
                if  object_name == "balll":

                    gg = distance_point_line(planned_trajectory[i][1][2],  planned_trajectory[i][1][3], object_position)
                    # Calculate the distance between the Jetbot and the ball and compare whether it is less than 20 cm or not
                    if gg <= 0.08:
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)

                        avoid_obstacle(jetbot_motor, 0)

                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        # get the Jetbot's current position and the orientation
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i am hier ")
                            break

                else:

                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)
                    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                    # get the Jetbot's current position and the orientation
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    if flag:
                        print("i am hier ")
                        break

            # get the Jetbot's current position and the orientation
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
        # go around a circle
        if planned_trajectory[i][0] == "kreis" + merged_sorted[j][0]:

            # get the Jetbot's current position and the orientation
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
            o2 = [planned_trajectory[i][1][0], planned_trajectory[i][1][1]]

            # get the required direction
            start = position[0:2]
            # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
            v1 = o2 - start
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

            turn_to_direction(jetbot_motor,direction)
            turn_to_direction(jetbot_motor, direction)
            object_name, object_score, object_center, object_size =get_detection()
            # turn clockwise or counterclockwise
            if merged_sorted[j][1][2] == "BLUE" :
                turn_counterclockwise(jetbot_motor, np.pi *2  )
            else:
                turn_clockwise(jetbot_motor, np.pi *2 )

            # get the Jetbot's current position and the orientation
            position, orientation =basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])

        # drive a curve
        if planned_trajectory[i][0] == "kurve" + merged_sorted[j][0]:
            # get the Jetbot's current position and the orientation
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
            # clockwise or counterclockwise
            if i != range(len(planned_trajectory))[-1]:
                if planned_trajectory[i - 2][1][2] > planned_trajectory[i + 1][1][2]:
                    if position[1] <= (planned_trajectory[i][1][1]):
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1] ) - 0.125]
                    else:
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) + 0.125]

                    # get the required direction
                    start = position[0:2]
                    ##
                    # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
                    v1 = o2 - start
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
                    ####

                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)
                    for p in range(7):
                        object_name, _, object_position, _ = get_object_position(jetbot_motor)
                        # get the Jetbot's current position and the orientation
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        # get the Jetbot's current position and the orientation
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i am hier")
                            break
                else:
                    if position[1] <= (planned_trajectory[i][1][1] ) :
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1] ) - 0.125]
                    else:
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) + 0.125]

                    # get the required direction
                    start = position[0:2]
                    ##
                    # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
                    v1 = o2 - start
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
                    ####

                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)

                    for p in range(7):
                        object_name, _, object_position, _ = get_object_position(jetbot_motor)
                        # get the Jetbot's current position and the orientation
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        # get the Jetbot's current position and the orientation
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i am hier ")
                            break
            else:
                if position[1] <= (planned_trajectory[i][1][1]):
                    o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) -0.125]
                else:
                    o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) + 0.125]

                # get the required direction
                start = position[0:2]
                ##
                # start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
                v1 = o2 - start
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
                ####

                turn_to_direction(jetbot_motor, direction)
                turn_to_direction(jetbot_motor, direction)
                for p in range(7):
                    object_name, _, object_position, _ = get_object_position(jetbot_motor)
                    # get the Jetbot's current position and the orientation
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)
                    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                    # get the Jetbot's current position and the orientation
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    if flag:
                        print("i am hier")
                        break

            # get the Jetbot's current position and the orientation
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])

####
# go bach to the start point
####
# get the Jetbot's current position and the orientation
position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])
o2=[planned_trajectory[-2][1][2], planned_trajectory[-2][1][3]]
# get the required direction
start = position[0:2]
##
# start: [x1, y1]   , end: [x2, y2]  , position: [x3, y3]
v1 = o2 - start
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
####

turn_to_direction(jetbot_motor, direction)
turn_to_direction(jetbot_motor, direction)

o2=[planned_trajectory[-2][1][2], planned_trajectory[-2][1][3]]
for p in range(7):
    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
    if flag:
        print("i am hier ")
        break


# get the Jetbot's current position and the orientation
position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])
o2=[planned_trajectory[-1][1][2], planned_trajectory[-1][1][3]]
for p in range(7):
    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
    if flag:
        print("i am hier ")
        break

# get the Jetbot's current position and the orientation
position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])

print(pos_real)

# save the curret position list in a text file
dateiname = 'pos_real.txt'
save_list(pos_real, dateiname)


