#!/usr/bin/env python
from basic_move import *
from object_localization import *

object_name, object_score, object_position = get_object_position()
destination = [object_position[0][0]+0.1, object_position[0][1]]
flag = False
while not flag:
    flag = linear_motion(jetbot_motor, destination)
    print('flag is:', flag)
turn_to_direction_advanced(jetbot_motor, np.pi/2)
avoid_obstacle(jetbot_motor, side=1)
print('avoid the obstacle successfully!')