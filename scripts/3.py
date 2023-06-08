#!/usr/bin/env python3
import os
import select
import sys
import rospy
import time


from jetbot import Robot
a=time.time()
robot =Robot()
robot.left(speed=0.3)
while True:
    b=time.time()
    if (b-a) > 2:
        robot.stop()
        break
robot.set_motors(0.2,0.6)
while True:
    b=time.time()
    if (b-a) > 6:
        robot.stop()
        break


