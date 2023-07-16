from object_localization import get_object_position
from object_localization import find_object
from object_localization import get_objects
from basic_move import *
import rospy
import numpy as np
from basic_move import turn_to_direction
import math
#from apriltag import get_apriltag
from motors_waveshare import MotorControllerWaveshare
import matplotlib.pyplot as plt
import os

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



"""
Target Point
"""
a=[0]*8
a[0]=[1.2,0.3]
a[1]=[1.2,0.6]
a[2]=[0.3,0.6]
a[3]=[0.3,0.9]
a[4]=[1.2,0.9]
a[5]=[1.2,1.2]
a[6]=[0.3,1.2]
a[7]=[0.1,0.1]
#Initialize variables
plot_position=[]
plot_name=[]
jetbot_motor = MotorControllerWaveshare()
timestamp = rospy.Time.now().to_sec()
save_variable(timestamp, 'time.pkl')
save_variable([0.1, 0.1], 'position.pkl')

end=[0.3,0.3] # Starting point
linear_motion(jetbot_motor, end) # move to Starting point
turn_to_direction(jetbot_motor, 3*np.pi/2) # turn to initialized starting dirction
position, _ = get_apriltag(jetbot_motor) # get position of Jetbot
objects= get_objects() # get other information of Jetbot
if objects!=[]:# Detecting whether an object is detected
    nearest_object= find_nearest_cube(objects)
    object_distance =  nearest_object.distance

for i in range(8):
    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)#The distance from the point where the obstacle is detected to the route of the Jetbot
    while distance_to_destination > 0.08 :# Terminate the loop if the distance to the target point is greater than 8cm.
        position, _ = get_apriltag(jetbot_motor)# get position of Jetbot
        if position[0] > 1.35 and position[0] < 0.08:# Determine whether it is in an unreasonable position
            position, _ = get_apriltag(jetbot_motor)
        else:
            objects = get_objects()
            object_angle= find_object()
            #Detect if an object is detected and record its position
            if objects !=[]:
                nearest_object = find_nearest_cube(objects)
                object_distance = nearest_object.distance
                print("*******************************object_distance:",object_distance)
                print("*******************************object_name:", nearest_object.name)
                object_position = nearest_object.position
                name=nearest_object.name
                distance = distance_point_line(position, a[i], object_position)
                # Start recording the object's position if it is detected within a distance of less than 30cm
                if object_distance <0.3:
                    if name =='ball' :
                        plot_name.append(nearest_object.name)
                        plot_position.append(nearest_object.position)
                    elif name not in plot_name:
                        plot_name.append(nearest_object.name)
                        print("************************plot_name", plot_name)

                        plot_position.append(nearest_object.position)
                        print("************************plot_position",plot_position)

                # Determine whether the obstacle avoidance conditions are met
                if object_distance<0.2 and distance< 0.1 and ((object_position[0] > 0.2 and object_position[0] < 1.2) and  (object_position[1]>0.2 and object_position[1]<1.2)):
                    print("*********************start avoid obstacle***************************")
                    object_angle = nearest_object.horizon_angle
                    print("*******************************object_angle:", object_angle)

                    #obstacle avoidance strategy
                    if (object_position[0] > 1.15 or object_position[0]<0.4) and i < 7:
                        break
                    elif i in range(2,4,6):
                        if object_position[0] > 1.15 or object_position[0]<0.4:
                            break
                    elif object_angle > 5:
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
                            end =[object_position[0]-0.15,object_position[1]]
                            linear_motion(jetbot_motor, end)
                            continue
                # If there are no obstacles, move towards the target point
                else:
                    linear_motion_with_desired_time(jetbot_motor, a[i], 1)
                    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
            else:
                linear_motion_with_desired_time(jetbot_motor, a[i], 1)
                distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
#return to origin
turn_to_direction(jetbot_motor,[0,0])
forwards_distance(jetbot_motor,0.15)


"""
To save the object's position and color information
"""

file_list_position="./list_position.npy"
if os.path.exists(file_list_position):

    os.remove(file_list_position)
np.save(file_list_position,plot_position)
file_list_name = "./list_name.npy"
if os.path.exists(file_list_position):

   os.remove(file_list_position)
np.save(file_list_name,plot_name)
