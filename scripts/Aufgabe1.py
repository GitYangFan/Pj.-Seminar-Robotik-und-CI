from object_localization import get_object_position
from object_localization import find_object
from object_localization import get_objects
from basic_move import linear_motion
from basic_move import turn_clockwise
from basic_move import turn_counterclockwise
from basic_move import avoid_obstacle
import rospy
import numpy as np
from basic_move import turn_to_direction
import math
from apriltag import get_apriltag
from motors_waveshare import MotorControllerWaveshare
import matplotlib.pyplot as plt


jetbot_motor = MotorControllerWaveshare()

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

def approach_obstacle(obstacle_position):
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
    p_x = (-DISTANCE_TO_OBSTACLE * uVec[0]) + object_position[0]
    p_y = (-DISTANCE_TO_OBSTACLE * uVec[1]) + object_position[1]
    p = [p_x, p_y]
    linear_motion(jetbot_motor, p)


def plot(position, color, marker):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, 1.485)
    ax.set_ylim(0, 1.485)
    ax.set_title('Objects')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    for pos, color, marker in zip(position, color, marker):
        x, y = pos
        ax.scatter(x, y, color=color, marker=marker)
    plt.show()



position,_ =get_apriltag()

#global a
a = position[0]
d = position[1]
b=0
i=-1
for c in range(0,6):
    b=b+1
    i = math.pow(-1, b+1)
    for e in range(0,6):
        a=a+0.2*i
        end= [a,position[1]]
        object= get_objects()
        nearest_object= find_nearest_cube(object)
        object_distance =  nearest_object.distance
        object_position =nearest_object.position
        if object_distance< 0.2:
            i=i+1
        if object_distance < 0.2 and a< 1.2 :
           # rospy.init_node('object_localization')
            #_, _, object_position,_ = get_object_position()  # get_object_position()# object_position = objects[2].object_position
            #end2 = [object_position[0] - 0.1, object_position[1] - 0.1]
            #inear_motion(jetbot_motor, end2)
            approach_obstacle(object_position)
            object = get_objects()
            nearest_object = find_nearest_cube(object)
            #object_name, _, object_position,_ = get_object_position()

            plot_position[i]=nearest_object.position
            if cube in nearest_object.name:
                marker[i] = 's'
                color[i]=nearest_object.name[5:]
            elif ball in nearest_object.name:
                 marker[i]='o'
                 color[i]='gray'

            turn_to_direction(jetbot_motor, -math.atan(object_position[0] / object_position[1]))
            turn_clockwise(jetbot_motor, np.pi / 2)
            position, _ = get_apriltag()
            end=[position[0]+0.2,d]
            linear_motion(jetbot_motor, end)
            turn_to_direction(jetbot_motor, 3 / 2 * np.pi)
            a=position[0]


        elif object_distance < 0.1 and a > 1.2 :
            object_name, _, object_position, _ = get_object_position()
            plot_position[i] = object_position
            if cube in object_name:
                marker[i] = 's'
                color[i] = object_name[5:]
            elif ball in object_name:
                marker[i] = '0'
                color[i] = 'gray'
        else:
            linear_motion(jetbot_motor, end)


    d=d+0.2
    position,_ = get_apriltag()
    if position[0] > 1.2:
        turn_clockwise(jetbot_motor, 0)# Rotate pi/2 in place to left
        #_, _, object_distance = find_object()
        object = get_objects()
        nearest_object = find_nearest_cube(object)
        object_distance = nearest_object.distance
        object_position = nearest_object.position
        if object_distance <0.2:
           # _, _, object_position, _ = get_object_position()  # get_object_position()# object_position = objects[2].object_position
            #end1 = [object_position[0] - 0.1, object_position[1] - 0.1]
            #linear_motion(jetbot_motor, end1)
            approach_obstacle(object_position)
            turn_to_direction(jetbot_motor, -math.atan(object_position[0] / object_position[1]))
            turn_clockwise(jetbot_motor, np.pi / 2)
            position, _ = get_apriltag()
            end1=[position[0],position[1]+0.2]
            linear_motion(jetbot_motor, end1)
            turn_clockwise(jetbot_motor, 0)
        else:
            end1=[a,d]
            linear_motion(jetbot_motor, end1)
            turn_clockwise(jetbot_motor,0)
    elif position[0] < 0.2:
        turn_counterclockwise(jetbot_motor, 0)  # Rotate pi/2 in place to left
        #_, _, object_distance = find_object()
        object = get_objects()
        nearest_object = find_nearest_cube(object)
        object_distance = nearest_object.distance
        object_position = nearest_object.position
        if object_distance < 0.2:
           # _, _, object_position, _ = get_object_position()  # get_object_position()# object_position = objects[2].object_position
            #end1 = [object_position[0] - 0.1, object_position[1] - 0.1]
            #linear_motion(jetbot_motor, end1)
            approach_obstacle(object_position)
            turn_to_direction(jetbot_motor, -math.atan(object_position[0] / object_position[1]))
            turn_counterclockwise(jetbot_motor, np.pi / 2)
            position, _ = get_apriltag()
            end1 = [position[0], position[1] + 0.2]
            linear_motion(jetbot_motor, end1)
            turn_counterclockwise(jetbot_motor, 0)
        else:
            end1 = [a, d]
            linear_motion(jetbot_motor, end1)
            turn_counterclockwise(jetbot_motor, 0)

#end3=[a,d]
#_, _, object_distance = find_object()
#if object_distance<0.2:

#    avoid_obstacle(jetbot_motor, left)
#linear_motion(jetbot_motor, end3)

















# 检查物体是否已经被检测过
#if not is_object_detected(x, y):
            # 物体未被检测过，执行相应操作
            # ...
            # 将物体位置存储到已检测物体列表中
 #           detected_objects.append((x, y))

# 检查物体是否已被检测过
#def is_object_detected(x, y):
 #   for obj in detected_objects:
  #      if abs(x - obj[0]) < 10 and abs(y - obj[1]) < 10:
   #         return True
    #return False