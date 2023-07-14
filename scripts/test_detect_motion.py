#!/usr/bin/env python
from basic_move import *
from object_localization import *
from motors_waveshare import MotorControllerWaveshare

jetbot_motor = MotorControllerWaveshare()
#rospy.init_node('object_localization')
object_name, object_score, object_position, object_distance_horizon = get_object_position()
destination = [object_position[0][0]+0.1, object_position[0][1]]
flag = False
while not flag:
    flag = linear_motion(jetbot_motor, destination)
    print('flag is:', flag)
turn_to_direction(jetbot_motor, np.pi/2)
avoid_obstacle(jetbot_motor, side=1)
print('avoid the obstacle successfully!')
