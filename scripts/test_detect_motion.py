#!/usr/bin/env python
from basic_move import *
from object_localization import *
from motors_waveshare import MotorControllerWaveshare
jetbot_motor = MotorControllerWaveshare()

# example code for detecting and avoiding the obstacle automatically
object_name, object_score, object_position = get_object_position()  # step 1: get the object position
destination = [object_position[0][0]+0.1, object_position[0][1]]    # step 2: calculate the destination based on your strategy(10 cm away from object)
flag = False    # step 3: initialization of flag of linear motion
while not flag:
    flag = linear_motion(jetbot_motor, destination)     # step 4: do the linear motion using a loop, if get the destination, then break
    print('flag is:', flag)
turn_to_direction(jetbot_motor, np.pi/2)    # step 5: turn to the obstacle itself, prepared for avoiding
avoid_obstacle(jetbot_motor, side=1)        # step 6: call the function of avoid_obstacle
print('avoid the obstacle successfully!')