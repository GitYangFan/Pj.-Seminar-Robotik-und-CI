#!/usr/bin/env python
from basic_move import *
from object_localization import *
from motors_waveshare import MotorControllerWaveshare

# emergency stop for testing

jetbot_motor = MotorControllerWaveshare()
#rospy.init_node('object_localization')
stop(jetbot_motor)
print('Emergency stop!')
