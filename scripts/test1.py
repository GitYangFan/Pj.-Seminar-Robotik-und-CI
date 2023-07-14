from object_localization import get_object_position
from object_localization import find_object
from object_localization import get_objects
from basic_move import *
import rospy
import numpy as np
from basic_move import turn_to_direction
import math
# from apriltag import get_apriltag
from motors_waveshare import MotorControllerWaveshare
import matplotlib.pyplot as plt
from apriltag import save_variable
from apriltag import load_variable


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

def approach_obstacle(jetbot_motor, obstacle_position):
    """
    Approach the obstacle for avoiding

    Args:
        obstacle_position ([float, float]): the position of the obstacle need to avoid
    """
    DISTANCE_TO_OBSTACLE = 0.10
    position, _ = get_apriltag(jetbot_motor)
    vec = [obstacle_position[0] - position[0], obstacle_position[1] - position[1]]
    magnitude = math.sqrt(vec[0]**2 + vec[1]**2)
    uVec = [vec[0] / magnitude, vec[1] / magnitude]
    p_x = (-DISTANCE_TO_OBSTACLE * uVec[0]) + obstacle_position[0]
    p_y = (-DISTANCE_TO_OBSTACLE * uVec[1]) + obstacle_position[1]
    p = [p_x, p_y]
    linear_motion(jetbot_motor, p)


a=[0]*8
a[0]=[1.2,0.3]
a[1]=[1.2,0.6]
a[2]=[0.3,0.6]
a[3]=[0.3,0.9]
a[4]=[1.2,0.9]
a[5]=[1.2,1.2]
a[6]=[0.3,1.2]
a[7]=[0.2,0.2]

plot_position=[]
plot_name=[]

jetbot_motor = MotorControllerWaveshare()

timestamp = rospy.Time.now().to_sec()
save_variable(timestamp, 'time.pkl')
save_variable([0.1, 0.1], 'position.pkl')

turn_to_direction(jetbot_motor, 3*np.pi/2)
end=[0.2,0.3]
linear_motion(jetbot_motor, end)
turn_to_direction(jetbot_motor, 3*np.pi/2)
position, _ = get_apriltag(jetbot_motor)
objects= get_objects(jetbot_motor)
if objects!=[]:
    nearest_object= find_nearest_cube(objects)
    object_distance =  nearest_object.distance

for i in range(8):
    print("i=",i)
    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
    while distance_to_destination > 0.08 :
        position, _ = get_apriltag(jetbot_motor)
        if position[0] > 1.35 and position[0] < 0.08:
            position, _ = get_apriltag(jetbot_motor)
        else:
            objects = get_objects(jetbot_motor)
            object_angle= find_object()
            if objects !=[]:
                nearest_object = find_nearest_cube(objects)
                object_distance = nearest_object.distance
                print("*******************************object_distance:",object_distance)
                print("*******************************object_name:", nearest_object.name)
                object_position = nearest_object.position
                distance = distance_point_line(position, a[i], object_position)
                name=nearest_object.name
                if object_distance <0.35:
                    if name == 'ball':
                        plot_name.append(nearest_object.name)
                        plot_position.append(nearest_object.position)
                        print("************************plot_name", plot_name)
                    elif name not in plot_name:
                        plot_name.append(nearest_object.name)
                        print("************************plot_name", plot_name)

                        plot_position.append(nearest_object.position)
                        print("************************plot_position",plot_position)



                    #with open('output.txt', 'w') as file:
                     #   for item in plot_position:
                      #      line = f"{item.position},{item.name}\n"
                       #     file.write(line)


                if object_distance<0.2 and distance< 0.1 and ((object_position[0] > 0.3 and object_position[0] < 1.2) and  (object_position[1]>0.2 and object_position[1]<1.2)):
                    print("*********************start avoid obstacle***************************")
                    object_angle = nearest_object.horizon_angle
                    print("*******************************object_angle:", object_angle)
                    if (object_position[0] > 1.1 or object_position[0]<0.4) and i<7 :
                
                        break
                   # elif i in [1, 3, 5]:
                    #    print("*********************in Y_Richtung***************************")
                     #   if object_position[0] > 1.15 or object_position[0]<0.4:
                      #      break

                    elif object_angle > 0.1745:
                        if i%2==0:
                            end = [object_position[0],object_position[1]-0.15]
                            linear_motion(jetbot_motor,end)
                        else:
                            end = [object_position[0]+0.15, object_position[1]]
                            linear_motion(jetbot_motor, end)
                            continue

                    else:
                        if i % 2 == 0:

                            end=[object_position[0],object_position[1]+0.15]
                            linear_motion(jetbot_motor, end)
                        else:
                            end =[object_position[1]-0.15,object_position[1]]
                            linear_motion(jetbot_motor, end)
                            continue

               # elif (object_position[0] > 1.15) :
                #    break


                else:
                    linear_motion_with_desired_time(jetbot_motor, a[i], 1)
                    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)


            else:
                linear_motion_with_desired_time(jetbot_motor, a[i], 1)
                distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)


turn_to_direction(jetbot_motor,3*np.pi/4)
forwards_distance(jetbot_motor,0.15)

file_list_position="./list_position.npy"
if os.path.exists(file_list_position):
    os.remove(file_list_position)
np.save(file_list_position,plot_position)
file_list_name = "./list_name.npy"
if os.path.exists(file_list_name):
    os.remove(file_list_name)
np.save(file_list_name,plot_name)
