#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import motors_waveshare


def motor_control():
    pub = rospy.Publisher('~cmd_str', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    motor_test = String()
    while not rospy.is_shutdown():
        motor_test = "left"
        rospy.loginfo(motor_test)
        pub.publish(motor_test)
        rate.sleep()


if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        pass
