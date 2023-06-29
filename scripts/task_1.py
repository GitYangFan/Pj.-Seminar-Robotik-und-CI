#!/usr/bin/env python
from basic_move import *
from object_localization import *
from motors_waveshare import MotorControllerWaveshare

jetbot_motor = MotorControllerWaveshare()

a = [0] * 12
a[0] = [1.2, 0.2]
a[1] = [1.2, 0.4]
a[2] = [0.2, 0.4]
a[3] = [0.2, 0.6]
a[4] = [1.2, 0.6]
a[5] = [1.2, 0.8]
a[6] = [0.2, 0.8]
a[7] = [0.2, 1.0]
a[8] = [1.2, 1.0]
a[9] = [1.2, 1.2]
a[10] = [0.2, 1.2]
a[11] = [0.2, 0.2]

for i in range(12):
    print('Searching for the point:', i, 'position:', a[i], '-----------------------------------------')
    flag = False
    while not flag:
        flag = linear_motion_with_desired_time(jetbot_motor, a[i], 2)
        object_name, object_score, object_position, object_distance_horizon = get_object_position()
print('Finished! Returned to the start point. ')
