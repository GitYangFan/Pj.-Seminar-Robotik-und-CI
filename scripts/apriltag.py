import numpy as np
import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray

apriltag = []

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = -math.atan2(t3, t4)
    if yaw_z < 0:
        yaw_z = 2*np.pi - yaw_z

    euler = [roll_x, pitch_y, yaw_z]
    return euler  # in radians

def tag_callback(data):
    global apriltag
    if data.detections != []:
        apriltag = data.detections[0].pose.pose.pose
        # print(apriltag.position.x)
        # print(data.detections[0].pose.pose.pose)

def get_apriltag():
    rospy.init_node('apriltag_listener')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if apriltag !=[]:
            # print(apriltag)
            position = [apriltag.position.x, apriltag.position.y, apriltag.position.z]
            orientation = euler_from_quaternion([apriltag.orientation.x, apriltag.orientation.y, apriltag.orientation.z, apriltag.orientation.w])
            return position, orientation
        rospy.sleep(0.1)




"""
-------------------- test ----------------------------
"""


position, orientation = get_apriltag()
print('position:', position, 'orientation:',orientation)
