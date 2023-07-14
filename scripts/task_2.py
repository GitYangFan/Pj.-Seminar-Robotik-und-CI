import math
import numpy as np
#import matplotlib.pyplot as plt
from matplotlib import pyplot as plt, patches
from basic_move import linear_motion
from basic_move import *
from motors_waveshare import MotorControllerWaveshare
from detection_results import get_detection
from object_localization  import *
import basic_move



planned_trajectory =[('pointw1', [0.1, 0.1, 0.5, 0.475]), ('kreisw1', [0.5, 0.6, 0.125]), ('kurvew1', [0.375, 0.6]), ('pointw3', [0.375, 0.6, 0.4, 0.975]), ('kreisw3', [0.4, 1.1, 0.125]), ('kurvew3', [0.525, 1.1]), ('pointw6', [0.525, 1.1, 0.975, 1.1]), ('kreisw6', [1.1, 1.1, 0.125]), ('pointw2', [0.975, 1.1, 1.0, 0.825]), ('kreisw2', [1.0, 0.7, 0.125]), ('kurvew2', [1.125, 0.7]), ('pointw5', [1.125, 0.7, 1.2, 0.425]), ('kreisw5', [1.2, 0.3, 0.125]), ('kurvew5', [1.075, 0.3]), ('pointw4', [1.075, 0.3, 0.8, 0.425]), ('kreisw4', [0.8, 0.3, 0.125]), ('kurvew4', [0.675, 0.3]), ('start_point90', [0.675, 0.3, 0.675, 0.1]), ('start_point', [0.675, 0.1, 0.1, 0.1])]


merged_sorted =[('w1', [0.5, 0.6, 'BLUE']), ('w3', [0.4, 1.1, 'BLUE']), ('w6', [1.1, 1.1, 'RED']), ('w2', [1.0, 0.7, 'BLUE']), ('w5', [1.2, 0.3, 'RED']), ('w4', [0.8, 0.3, 'RED'])]

#save a list in text data
def save_list(liste, dateiname):
    with open(dateiname, 'w') as datei:
        for element in liste:
            datei.write(str(element) + '\n')



# Function to calculate the distance between two points
def calculate_distance(o1, o2):
    x1, y1 = o1
    x2, y2 = o2
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance


def merge_dicts(dict1, dict2):
    merged_dict = dict1.copy()  # Create a copy of dict1

    for key, value in dict2.items():
        if key in merged_dict:
            if isinstance(value, dict) and isinstance(merged_dict[key], dict):
                merged_dict[key] = merge_dicts(merged_dict[key], value)
            elif isinstance(value, list) and isinstance(merged_dict[key], list):
                merged_dict[key].extend(value)
            else:
                merged_dict[key] = value
        else:
            merged_dict[key] = value

    return merged_dict
#calculate the angel of a thriangel

def calculate_angles(a, b, c):
    # Calculate the angles using the law of cosines
    angle_a = math.degrees(math.acos((b**2 + c**2 - a**2) / (2 * b * c)))
    angle_b = math.degrees(math.acos((c**2 + a**2 - b**2) / (2 * c * a)))
    angle_c = 180 - angle_a - angle_b  # The sum of angles in a triangle is 180 degrees

    return angle_a, angle_b, angle_c

#calculate_sides
def calculate_sides(hypotenuse, angle):
    # Berechnung des Kathetenlaeangen mit dem gegebenen Winkel
    adjacent = hypotenuse * math.cos(math.radians(angle))
    opposite = hypotenuse * math.sin(math.radians(angle))

    return adjacent, opposite

def distance_to_line(point, line):
    x, y = point
    x1, y1 = line[0]
    x2, y2 = line[1]

    numerator = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

    distance = numerator / denominator
    return distance
#intersection_point between a line and a point
def intersection_point(point, line):
    x, y = point
    x1, y1 = line[0]
    x2, y2 = line[1]

    # ueberpruefen, ob die Linie vertikal ist, um eine Division durch 0 zu vermeiden
    if x1 == x2 or y1 == y2:
        intersection_x = x1
        intersection_y = y
    else:
        m = (y2 - y1) / (x2 - x1)  # Berechnung der Steigung der Linie
        b = y1 - m * x1  # Berechnung des y-Achsenabschnitts

        # ueberpruefen, ob der Punkt auf der Linie liegt
        if y == m * x + b:
            intersection_x = x
            intersection_y = y
        else:
            # Berechnung des Schnittpunkts
            intersection_x = (y - b) / m
            intersection_y = y

    return intersection_x, intersection_y



pos_real = []
cube_color_real = []
next_next_ob= 0
jetbot_motor = MotorControllerWaveshare()

for j in range(len(merged_sorted)):
    next_next_ob = next_next_ob + 1
    last_ob = merged_sorted[-1][0]
    # next next position
    if merged_sorted[j][0] != last_ob:
        # ggg = merged_sorted[next_next_ob][0]
        jjj = merged_sorted[j + 1]

    for i in range(len(planned_trajectory)):
        if planned_trajectory[i][0] == "point" + merged_sorted[j][0]:
            #umdrehen und erkennen ob eine Kugel da ist oder nicht wenn nicht kann der Jetbot der erste 20 cm fahren wenn ja halb kreis undrehen
            #print(planned_trajectory[i][2])
            #print(planned_trajectory[i][3])

            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
            o2=[planned_trajectory[i][1][2], planned_trajectory[i][1][3]]

            #Abstand zwischen dem Jetbot und dem Objekt
            #distance_ob_total = calculate_distance(position[0:2], o2)
            #pos_stop= distance_ob_total/0.2
            #rounded_pos_stop = math.ceil(pos_stop)

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

            turn_to_direction(jetbot_motor, direction)
            turn_to_direction(jetbot_motor, direction)
            #detektion ob eine Kugel da ist
            #get_detection()
            object_name,_, object_position,_ = get_object_position(jetbot_motor)
            #adjacent, opposite = calculate_sides(0.30, direction)
            #print("opposite: ", opposite)
            #print("adjacent: ", adjacent)
            #end = [object_position[0], object_position[1]]
            for p in  range(7):
                object_name, _, object_position, _ = get_object_position(jetbot_motor)
                position, orientation = basic_move.get_apriltag(jetbot_motor)
                pos_real.append(position[0:2])
                if  object_name == "balll":

                    gg = distance_point_line(planned_trajectory[i][1][2],  planned_trajectory[i][1][3], object_position)
                    #die distance zwischen dem Jetbot und die Kugel recchnen und vergleichen ob es Kleiner als 20 cm oder nicht
                    if gg <= 0.08:
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)

                        avoid_obstacle(jetbot_motor, 0)

                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i bin hier ")
                            break

                else:

                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)
                    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    if flag:
                        print("i bin hier ")
                        break
            #linear_motion(jetbot_motor, o2)
            print("Ziel_punkt:", o2)

            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])

        if planned_trajectory[i][0] == "kreis" + merged_sorted[j][0]:

            #orientation[2]
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
            o2 = [planned_trajectory[i][1][0], planned_trajectory[i][1][1]]

            #
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
            # get the angel?

            turn_to_direction(jetbot_motor,direction)
            turn_to_direction(jetbot_motor, direction)
            object_name, object_score, object_center, object_size =get_detection()
            if merged_sorted[j][1][2] == "BLUE" :
                turn_counterclockwise(jetbot_motor, np.pi *2  )
            else:
                turn_clockwise(jetbot_motor, np.pi *2 )




            position, orientation =basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])

        if planned_trajectory[i][0] == "kurve" + merged_sorted[j][0]:
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])
            #o2=[planned_trajectory[i][1][2], planned_trajectory[i][1][3]]
            #o2 = [merged_sorted[j][1][0], merged_sorted[j][1][1]]

            # mit Uhrzeigesin oder gegen!
            if i != range(len(planned_trajectory))[-1]:
                if planned_trajectory[i - 2][1][2] > planned_trajectory[i + 1][1][2]:
                    #turn_clockwise(jetbot_motor, np.pi / 2)
                    if position[1] <= (planned_trajectory[i][1][1] ) :
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1] ) - 0.125]
                    else:
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) + 0.125]


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
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i bin hier ")
                            break
                else:
                    #turn_counterclockwise(jetbot_motor, np.pi/ 2)
                    if position[1] <= (planned_trajectory[i][1][1] ) :
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1] ) -0.125]
                    else:
                        o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) +0.125]

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
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        turn_to_direction(jetbot_motor, direction)
                        turn_to_direction(jetbot_motor, direction)
                        flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                        position, orientation = basic_move.get_apriltag(jetbot_motor)
                        pos_real.append(position[0:2])
                        if flag:
                            print("i bin hier ")
                            break
            else:
                # turn_counterclockwise(jetbot_motor, np.pi/ 2)
                if position[1] <= (planned_trajectory[i][1][1]):
                    o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) -0.125]
                else:
                    o2 = [planned_trajectory[i][1][0], (planned_trajectory[i][1][1]) + 0.125]

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
                #turn_counterclockwise(jetbot_motor, np.pi / 2)
                for p in range(7):
                    object_name, _, object_position, _ = get_object_position(jetbot_motor)
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    turn_to_direction(jetbot_motor, direction)
                    turn_to_direction(jetbot_motor, direction)
                    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
                    position, orientation = basic_move.get_apriltag(jetbot_motor)
                    pos_real.append(position[0:2])
                    if flag:
                        print("i bin hier ")
                        break
            position, orientation = basic_move.get_apriltag(jetbot_motor)
            pos_real.append(position[0:2])


position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])
#o2=[merged_sorted[-1][1][0], merged_sorted[-1][1][1]]
#
o2=[planned_trajectory[-2][1][2], planned_trajectory[-2][1][3]]
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
        print("i bin hier ")
        break


position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])
o2=[planned_trajectory[-1][1][2], planned_trajectory[-1][1][3]]
for p in range(7):
    flag = linear_motion_with_desired_time(jetbot_motor, o2, 2)
    if flag:
        print("i bin hier ")
        break
position, orientation = basic_move.get_apriltag(jetbot_motor)
pos_real.append(position[0:2])

"""
planned_trajectory=
{'pointw2': [10, 10, 22, 12.5], 'kreisw2': [22, 20, 7.5], 'kurvew2': [29.5, 20], 'pointw5': [29.5, 20, 65, 82.5], 'kreisw5': [65, 90, 7.5], 'kurvew5': [72.5, 90], 'pointw3': [72.5, 90, 112.5, 120], 'kreisw3': [120, 120, 7.5], 'pointw4': [112.5, 120, 120, 77.5], 'kreisw4': [120, 70, 7.5], 'kurvew4': [112.5, 70], 'pointw6': [112.5, 70, 80, 67.5], 'kreisw6': [80, 60, 7.5], 'kurvew6': [87.5, 60], 'pointw1': [87.5, 60, 120, 52.5], 'kreisw1': [120, 45, 7.5], 'kurvew1': [112.5, 45], 'start_point90': [120, 37.5, 120, 10], 'start_point': [120, 10, 10, 10]}

"""
print(pos_real)
print(cube_color_real)
dateiname = 'pos_real.txt'
save_list(pos_real, dateiname)

dateiname = 'cube_color_real.txt'
save_list(cube_color_real, dateiname)
# Set plot
# plt.xlim(0, 149.5)
# plt.ylim(0, 149.5)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Arena')
#plt.legend()

# plt.grid()

# plt.show()
