#!/usr/bin/env python
import os
import time

os.system("gnome-terminal -e 'bash -c \" roscore; exec bash\"'")

a=time.time()
while True:
    b=time.time()
    if (b-a) > 4:
        a=b=0
        os.system("gnome-terminal -e 'bash -c \" rosrun jetbot_ros jetbot_motors.py; exec bash\"'")

        break



os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'left'")
a=time.time()

while True:
    b=time.time()
    if (b-a) > 1:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break

os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'forward'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 2:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break
os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'left'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 1:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break

os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'forward'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 2:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break
os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'left'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 1:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break

os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'forward'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 2:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break
os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'left'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 1:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break

os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'forward'")
a=time.time()
while True:
    b=time.time()
    if (b-a) > 2:
        os.system("rostopic pub /jetbot_motors/cmd_str std_msgs/String --once 'stop'")
        break
    

