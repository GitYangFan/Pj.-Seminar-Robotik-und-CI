import numpy as np
import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
import pickle
from basic_move import *
from motors_waveshare import MotorControllerWaveshare

apriltag = []
jetbot_motor = MotorControllerWaveshare()


def save_variable(var, filename):
    with open(filename, 'wb') as f:
        pickle.dump(var, f)

def load_variable(filename):
    with open(filename, 'rb') as f:
        var = pickle.load(f)
    return var


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
    w = -quaternion[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    if yaw_z < 0:  # Adjust the yaw angle range to [0,2*pi]
        yaw_z = 2 * np.pi + yaw_z

    euler = [roll_x, pitch_y, yaw_z]
    return euler  # in radians


def transformation(euler, position_relativ):
    # euler = euler_from_quaternion(quaternion)
    euler = [euler[0] / 3.14 * 180, euler[1] / 3.14 * 180, euler[2] / 3.14 * 180]
    angle_x = np.deg2rad(euler[0])
    angle_y = np.deg2rad(euler[1])
    angle_z = np.deg2rad(euler[2])
    rotation_matrix_x = np.array(
        [[1, 0, 0], [0, np.cos(angle_x), -np.sin(angle_x)], [0, np.sin(angle_x), np.cos(angle_x)]])
    rotation_matrix_y = np.array(
        [[np.cos(angle_y), 0, np.sin(angle_y)], [0, 1, 0], [-np.sin(angle_y), 0, np.cos(angle_y)]])
    rotation_matrix_z = np.array(
        [[np.cos(angle_z), -np.sin(angle_z), 0], [np.sin(angle_z), np.cos(angle_z), 0], [0, 0, 1]])
    rotation_matrix = np.dot(np.dot(rotation_matrix_z, rotation_matrix_y), rotation_matrix_x)
    position_real = -np.dot(rotation_matrix, position_relativ)
    # print(position_real)
    return position_real


def tag_callback(data):
    global apriltag
    if data.detections != []:
        apriltag = data.detections[0].pose.pose.pose
        # print(apriltag.position.x)
        # print(data.detections[0].pose.pose.pose
        # print('detection:',data.detections)


def get_apriltag():
    # rospy.init_node('apriltag_listener')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    # get previous position and time
    time_pre = load_variable('time.pkl')
    position = load_variable('position.pkl')

    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if apriltag != []:
            # print(apriltag)
            quaternion = [apriltag.orientation.x, apriltag.orientation.y, apriltag.orientation.z,
                          apriltag.orientation.w]
            position_relative = [apriltag.position.x, apriltag.position.y, apriltag.position.z]
            orientation = euler_from_quaternion(quaternion)
            position = transformation(orientation, position_relative)
            # print('position_relative:', position_relative, 'quaternion:',quaternion)
            if (0 < position[0] < 1.485) and (0 < position[1] < 1.485) and ():
                timestamp = rospy.Time.now()
                save_variable(timestamp, 'time.pkl')
                save_variable(position, 'position.pkl')
                return position, orientation
        if (rospy.Time.now()-time_pre) > 2:     # when the apriltag doesn't come for more than 2 seconds, then shake and test again
            shake_turn(jetbot_motor)
            time_pre = rospy.Time.now()
        rospy.sleep(0.1)


"""
-------------------- test ----------------------------
"""

# position, orientation = get_apriltag()
# print('position:', position, 'orientation:',orientation)
