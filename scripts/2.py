#!/usr/bin/env python
import os
import select
import sys
import rospy
import time
import platform


from motors_waveshare import MotorControllerWaveshare
from geometry_msgs.msg import Twist

os.system("gnome-terminal -e 'bash -c \" roscore; exec bash\"'")
time.sleep(3)


pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
#rospy.init_node('teleop_keyboard') 

motor_controller = MotorControllerWaveshare()
twist = Twist()
       

twist.linear.x = 0.0
twist.linear.y = 0.0
twist.linear.z = 0.0

twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = 1.2

pub.publish(twist)


time.sleep(3)

twist.angular.z = 0.0
pub.publish(twist)

time.sleep(1)

left_speed= 0.1
right_speed = 0.5
motor_controller.set_speed(left_speed,right_speed)
time.sleep(2)
motor_controller.set_speed(0,0)






