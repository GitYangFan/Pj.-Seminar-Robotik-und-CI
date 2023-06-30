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


jetbot_motor = MotorControllerWaveshare()

position,_ =get_apriltag()
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



position, _ = get_apriltag()
jetbot_motor = MotorControllerWaveshare()

for i in range(12):
    distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
    while distance_to_destination > 0.05 :
        position, _ = get_apriltag()
        if position[0] > 1.35 and position[0] < 0.1:
            position, _ = get_apriltag()
        else:
            linear_motion_with_desired_time(jetbot_motor, a[i],1.5)
            distance_to_destination = math.sqrt((position[0] - a[i][0]) ** 2 + (position[1] - a[i][1]) ** 2)
