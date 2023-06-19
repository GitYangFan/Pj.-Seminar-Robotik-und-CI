import math
import numpy as np

def Transformation (euler,position_relativ):
    euler = euler_from_quaternion(quaternion)
    euler = [euler[0]/3.14*180,euler[1]/3.14*180,euler[2]/3.14*180]
    angle_x = np.deg2rad(euler[0])
    angle_y = np.deg2rad(euler[1])
    angle_z = np.deg2rad(euler[2])
    rotation_matrix_x = np.array([[1, 0, 0], [0, np.cos(angle_x), -np.sin(angle_x)], [0, np.sin(angle_x), np.cos(angle_x)]])
    rotation_matrix_y = np.array([[np.cos(angle_y), 0, np.sin(angle_y)], [0, 1, 0], [-np.sin(angle_y), 0, np.cos(angle_y)]])
    rotation_matrix_z = np.array([[np.cos(angle_z), -np.sin(angle_z), 0], [np.sin(angle_z), np.cos(angle_z), 0], [0, 0, 1]])
    rotation_matrix = np.dot(np.dot(rotation_matrix_z,rotation_matrix_y),rotation_matrix_x)
    position_real = -np.dot(position_relativ,rotation_matrix)
    print(position_real)
    return position_real

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = float(quaternion[0])
    y = float(quaternion[1])
    z = float(quaternion[2])
    w = float(quaternion[3])
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

    euler = [roll_x, pitch_y, yaw_z]
    return euler  # in radians
position_relativ = [-0.14962,-0.013018,0.4723]
quaternion = [0.86737,0.046935,-0.08761,-0.48765]
euler=euler_from_quaternion(quaternion)
Transformation (euler,position_relativ)
