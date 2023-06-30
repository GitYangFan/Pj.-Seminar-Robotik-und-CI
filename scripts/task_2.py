import math
import numpy as np
#import matplotlib.pyplot as plt
from matplotlib import pyplot as plt, patches
from basic_move import linear_motion
from basic_move import *
from basic_move import stop
from motors_waveshare import MotorControllerWaveshare
from detection_results import get_detection
from object_localization  import *
import apriltag 

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


x1, y1 = [-0.1, 0.12], [0.1, 0.4]
x2, y2 = [0.1, 0.10], [0.3, 0.2]

cubes_rot_blue = {
    "w1": [1.20, 0.45],
    "w2": [0.42, 0.40],
    "w3": [1.20, 1.20],
    "w4": [1.20, 0.70],
    "w5": [0.65, 0.90],
    "w6": [0.80, 0.60]

}


#find out which cubes's positions are smaler aer bigger than 75
x_smaller75 = []
x_bigger75 = []

for x in cubes_rot_blue:
    j = cubes_rot_blue[x]
    if j[0] <= 0.75:
        x_smaller75.append(x)
    else:
        x_bigger75.append(x)

# separate the dict cubes_rot_blue to tow dict
x_smaller75_d = {}
x_bigger75_d = {}

for ob, value in cubes_rot_blue.items():

    if ob  in x_smaller75:
        x_smaller75_d[ob] = value
    if ob in x_bigger75:
        x_bigger75_d[ob] = value

# sort based on the first digits then on the second digits
#smaller than or equal 75
sorted_x_smaller75_d = dict(sorted(x_smaller75_d.items(), key=lambda x: (str(x[1][0]))))
sorted_y_smaller75_d = dict(sorted(sorted_x_smaller75_d.items(), key=lambda x: (str(x[1][1]))))

#bigger than 75
sorted_x_bigger75_d = dict(sorted(x_bigger75_d.items(), key=lambda x: (str(x[1][0])), reverse=True))
sorted_y_bigger75_d = dict(sorted(sorted_x_bigger75_d.items(), key=lambda x: (str(x[1][1])),reverse=True))

#merge the two dict together
#merged_sorted = {**sorted_y_smaller75_d, **sorted_y_bigger75_d}
merged_sorted = merge_dicts(sorted_y_smaller75_d, sorted_y_bigger75_d)

x1 = [0 , 1]
x2 = [0, 1]

#current_postion
r=0.1
b=0.1

#next_postion

#dict for the planned and "driven" trajectory
planned_trajectory = {}


# fig = plt.figure()
# ax = fig.add_subplot()

# Function to calculate the distance between two points
def calculate_distance(o1, o2):
    x1, y1 = o1
    x2, y2 = o2
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance






g=0
for i in merged_sorted:
    g = g + 1
    # next_postion
    j = merged_sorted[i]

    last_ob = list(merged_sorted.keys())[-1]
    last_ob_x_smaller75_d = list(sorted_y_smaller75_d.keys())[-1]
    last_ob_x_bigger75_d = list(sorted_y_bigger75_d.keys())[-1]
    fisrt_ob_x_bigger75_d = list(sorted_y_bigger75_d.keys())[0]

    # next next position
    if i != last_ob:
        gg = list(merged_sorted.keys())[g]

    jj = merged_sorted[gg]

    # -7.5 damit der JetBot am Kreis haelt und einen Kreis Faefrt und dann den Kreis verlaesst
    #aber muss nich angepasst werden dass wir nicht an den Wuerfeln anstossen

    if i in x_smaller75_d:
        #from the start point
        if r *100 in range(10, 20) and b*100 in range(10, 20):
            x1 = [r  , j[0]]
            x2 = [b , j[1] -0.075]

            if  j[0] >= jj[0]  :
                # current postion
                r = j[0] - 0.075
                b = j[1]

            if  j[0] < jj[0]  :
                # current postion
                r = j[0] + 0.075
                b = j[1]

        elif i != last_ob_x_smaller75_d :

            x1 = [r  , j[0] ]
            x2 = [b , j[1] -0.075 ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - 0.075
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + 0.075
                b = j[1]

        else:
            x1 = [r  , j[0] ]
            x2 = [b , j[1] -0.075 ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] -0.075
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + 0.075
                b = j[1]

    if i in x_bigger75_d:
        if i == fisrt_ob_x_bigger75_d:

            x1 = [r  , j[0] -0.075 ]
            x2 = [b , j[1] ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - 0.075
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + 0.075
                b = j[1]

        else:
            x1 = [r, j[0] ]
            x2 = [b, j[1] + 0.075 ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - 0.075
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + 0.075
                b = j[1]



    circle2 = plt.Circle((j[0], j[1]),0.075 , fill=False  , edgecolor='black', facecolor='none')





    #plt.plot(x1, x2, marker = 'o')
    #planned_trajectory.append

    planned_trajectory['point'+ i] = []
    planned_trajectory["point" + i] += [x1[0], x2[0]]
    planned_trajectory["point" + i] += [x1[1], x2[1]]
    planned_trajectory['kreis' + i] = [j[0], j[1], 0.075]

    if x1[1] != r and x2[1] != b:
        planned_trajectory['kurve' + i] = [r, b]

    # line = plt.plot(x1 , x2 )
    # plt.arrow(x1[0], x2[0], x1[1] - x1[0], x2[1]- x2[0],
    #           width=1, length_includes_head=True, head_width=2, color='red')
    #ax.add_patch(circle2)

    #add_arrow(line, ax, label='Path')
    """    ax.annotate("", xytext=( r,  j[0]), xy=( b , j [1] ),
    arrowprops = dict(arrowstyle="->", linewidth=3, color='k'), size = 40)"""

    # sample at 0, 1/3rd, and 2/3rd of curve
    t = np.linspace(0, 2*np.pi, 100)
    adx0, adx1 = 0, len(t) // 3
    adx2 = adx1 * 2

    arrow0 = x1[adx0 + 1], x2[adx0 + 1], x1[adx0 + 1] - x1[adx0], x2[adx0 + 1] - x2[adx0]

    # plt.arrow(*arrow0 , shape='full', lw=0, length_includes_head=True, head_width=0.12)




    if i == last_ob:
        x1 = [j[0],j[0]]
        x2 = [j[1]- 0.075, 0.1]
        planned_trajectory['start_point90' ] = []
        planned_trajectory["start_point90" ] += [x1[0], x2[0]]
        planned_trajectory["start_point90"] += [x1[1], x2[1]]


        # plt.plot(x1, x2, label='Trajectory')
        # plt.arrow( j[0] ,(j[1]- 7.5) , j[0] - j[0],10- (j[1]- 7.5),
        #           width=1, length_includes_head=True, head_width=2, color='red')
        #plt.arrow(prev_value[0], prev_value[1], value[0] - prev_value[0], value[1] - prev_value[1],
         #         width=0.1, length_includes_head=True, head_width=0.3, color='red')
        # ax.add_patch(circle2)

        x1 = [j[0],0.1]
        x2 = [0.1, 0.1]
        planned_trajectory['start_point' ] = []
        planned_trajectory["start_point" ] += [x1[0], x2[0]]
        planned_trajectory["start_point"] += [x1[1], x2[1]]

        # plt.plot(x1, x2, label='Trajectory')
        # plt.arrow(j[0], 10, 10 - j[0],10- 10,
        #           width=1, length_includes_head=True, head_width=2, color='red')


    #Drawing_colored_circle = plt.Circle((j[0], j[1]), 200)
#circle1 = patches.Circle((100, 600), radius=50)


next_next_ob= 0
jetbot_motor = MotorControllerWaveshare()
for j in merged_sorted:
    next_next_ob = next_next_ob +1
    # next next position
    if j != last_ob:
        ggg = list(merged_sorted.keys())[next_next_ob]

    jjj = merged_sorted[ggg]

    for i in planned_trajectory:
        if i == "point" + j:
            #umdrehen und erkennen ob eine Kugel da ist oder nicht wenn nicht kann der Jetbot der erste 20 cm fahren wenn ja halb kreis undrehen
            #print(planned_trajectory[i][2])
            #print(planned_trajectory[i][3])

            position, orientation = apriltag.get_apriltag()
            o2=[planned_trajectory[i][2], planned_trajectory[i][3] ]

            #Abstand zwischen dem Jetbot und dem Objekt
            distance_ob_total = calculate_distance(position[0:2], o2)
            pos_stop= distance_ob_total/0.2
            rounded_pos_stop = math.ceil(pos_stop)
            """
            # Sides of the triangle
            a = planned_trajectory[i][3] - position[1]
            b = planned_trajectory[i][2] - position[0]

            # calculate the angel for the Cube orientation
            angle_a, angle_b, angle_c = calculate_angles(a, b, distance_ob_total)
            """
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
            #detektion ob eine Kugel da ist
            #get_detection()
            object_name,_, object_position,_ = get_object_position()
            adjacent, opposite = calculate_sides(0.30, direction)
            print("opposite: ", opposite)
            print("adjacent: ", adjacent)
            end = [position[0] + opposite, position[1] + adjacent]
            linear_motion(jetbot_motor, end)




        if i == "kreis" + j:

            #orientation[2]
            position, orientation = apriltag.get_apriltag()
            o2=[planned_trajectory[i][0], planned_trajectory[i][1] ]
            """            
            # Sides of the triangle
            a = planned_trajectory[i][1] - position[1]
            b = planned_trajectory[i][0] - position[0]

            # calculate the angel for the Cube orientation
            angle_a, angle_b, angle_c = calculate_angles(a, b, distance_ob_total)
            """
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
            #oder machen wir die Richtung einfach 90 grad ?

            turn_to_direction(jetbot_motor,direction)
            object_name, object_score, object_center, object_size =get_detection()
            if object_name == "cube_blue" :
                #wir muessen entweder das Zentrum der Kreis eingeben !!!
                turn_counterclockwise(jetbot_motor, np.pi *2  )

            else:
                turn_clockwise(jetbot_motor, np.pi *2 )

        if i == "kurv" + j:
            position, orientation = apriltag.get_apriltag()
            o2=[planned_trajectory[i][0], planned_trajectory[i][1] ]
            """            
            # Sides of the triangle
            a = planned_trajectory[i][1] - position[1]
            b = planned_trajectory[i][0] - position[0]

            # calculate the angel for the Cube orientation
            #radian und in der anderer Richtung
            angle_a, angle_b, angle_c = calculate_angles(a, b, distance_ob_total)
            """
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

            # mit Uhrzeigesin oder gegen!
            if  planned_trajectory[i][2]  > planned_trajectory[jjj][2]:
                turn_clockwise(jetbot_motor, np.pi/2 )
            else:
                turn_counterclockwise(jetbot_motor, (3*np.pi)/2)





            #linear_motion( jetbot_motor, planned_trajectory[i][2],planned_trajectory[i][3]  )


"""
{'pointw2': [10, 10, 22, 12.5], 'kreisw2': [22, 20, 7.5], 'kurvew2': [29.5, 20], 'pointw5': [29.5, 20, 65, 82.5], 'kreisw5': [65, 90, 7.5], 'kurvew5': [72.5, 90], 'pointw3': [72.5, 90, 112.5, 120], 'kreisw3': [120, 120, 7.5], 'pointw4': [112.5, 120, 120, 77.5], 'kreisw4': [120, 70, 7.5], 'kurvew4': [112.5, 70], 'pointw6': [112.5, 70, 80, 67.5], 'kreisw6': [80, 60, 7.5], 'kurvew6': [87.5, 60], 'pointw1': [87.5, 60, 120, 52.5], 'kreisw1': [120, 45, 7.5], 'kurvew1': [112.5, 45], 'start_point90': [120, 37.5, 120, 10], 'start_point': [120, 10, 10, 10]}
"""
print(planned_trajectory)
# Set plot
# plt.xlim(0, 149.5)
# plt.ylim(0, 149.5)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Arena')
#plt.legend()

# plt.grid()

# plt.show()

#Turn to the direction
#has been tested
"""
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

"""
