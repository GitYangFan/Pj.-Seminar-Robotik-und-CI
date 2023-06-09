import math
def read_last_seven_numbers(file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            last_line = lines[-1].strip()
            numbers = last_line.split()
            last_seven_numbers = numbers[-7:]
            velocity=last_seven_numbers[:3]
            quaternion=numbers[-4:]
            print("v=", velocity)
            print("q=", quaternion)
            return quaternion
    except FileNotFoundError:
        print("File not found.")
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

# 调用示例
file_path = r'D:\TU-Darmstadt\siminar\1.txt.txt' # 替换为实际的文件路径
quaternion=read_last_seven_numbers(file_path)
euler = euler_from_quaternion(quaternion)
print(euler)



#r'D:\TU-Darmstadt\siminar\1.txt.txt'