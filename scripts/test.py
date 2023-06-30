import numpy as np
import tf
import math

"""
def test():
    global x
    x = 1
    y = 2
    return y

x = 2
y = test()
print(x,y)

z = -6%10
print(z)

object_size = [[0,0]] * 5
print(object_size)
"""

# def calculate_arena_pose(jetbot_position, jetbot_orientation, arena_origin_position, arena_origin_orientation):
#
#     # 构建Arena原点的四元数
#     arena_origin_quaternion = tf.transformations.quaternion_from_euler(
#         arena_origin_orientation[0],
#         arena_origin_orientation[1],
#         arena_origin_orientation[2]
#     )
#
#     # 构建Jetbot位置的四元数
#     jetbot_quaternion = tf.transformations.quaternion_from_euler(
#         jetbot_orientation[0],
#         jetbot_orientation[1],
#         jetbot_orientation[2]
#     )
#
#     # 计算Jetbot相对于Arena坐标系的位置和方向
#     arena_position = tf.transformations.quaternion_multiply(
#         tf.transformations.quaternion_inverse(arena_origin_quaternion),
#         tf.transformations.quaternion_multiply(jetbot_quaternion, trans)
#     ) + arena_origin_position
#
#     arena_orientation = tf.transformations.quaternion_multiply(
#         arena_origin_quaternion,
#         jetbot_quaternion
#     )
#
#     return arena_position, arena_orientation
#
# # 在Jetbot坐标系下的Arena原点位置和方向
# arena_origin_position_jetbot = np.array([1.58, 0.168, -0.409])
# arena_origin_orientation_jetbot = np.array([-0.3428, 0.66056, -0.59709])
#
# # Jetbot坐标系下的Jetbot位置和方向
# jetbot_position = np.array([0, 0, 0])
# jetbot_orientation = np.array([0, 0, 0])  # 使用零方向，即与Jetbot坐标系相同
#
# # 计算Arena坐标系下Jetbot的位置和方向
# arena_position, arena_orientation = calculate_arena_pose(
#     jetbot_position,
#     jetbot_orientation,
#     arena_origin_position_jetbot,
#     arena_origin_orientation_jetbot
# )
#
# # 打印结果
# print("在Arena坐标系下的Jetbot位置：", arena_position)
# print("在Arena坐标系下的Jetbot方向：", arena_orientation)

# test = [1, 2, 3]
# print(test[0:2])
"""
v1 = [1, 1]
if v1[0] < 0:
    if v1[1] > 0:
        direction = np.arctan(-v1[0] / v1[1])
    else:
        direction = np.pi - np.arctan(v1[0] / v1[1])
else:
    if v1[1] < 0:
        direction = np.pi + np.arctan(-v1[0] / v1[1])
    else:
        direction = 2 * np.pi - np.arctan(v1[0] / v1[1])
print(direction / np.pi * 180)

v2 = [1, 0.5]
cross_product = v1[0] * v2[1] - v1[1] * v2[0]
print(cross_product)
"""


# test = []
# length = len(test)
# distance = [None]*length
# print(length)
# print(distance)
# # print(range(len(test)))
# for i in range(length):
#     print(i)


# def distance_point_line(line_start, line_end, point):
#     """
#     Parameters
#     ------------
#     line_start: The start point of the line
#     line_end: The end point of the line
#     point: The third point
#
#     Returns
#     ------------
#     """
#     numerator = (line_end[1] - line_start[1]) * point[0] - (line_end[0] - line_start[0]) * point[1] + line_end[0] * \
#                 line_start[
#                     1] - line_end[1] * line_start[0]
#     denominator = math.sqrt((line_end[1] - line_start[1]) ** 2 + (line_end[0] - line_start[0]) ** 2)
#     distance = numerator / denominator
#     return distance
#
#
# test = distance_point_line([0, 0], [3, 4], [0, 1])
# print(test)


# test = [1, 2, 3, 4, 5]
# idx_cubeinbase = [2,4]
#
# if idx_cubeinbase:
#     for i in range(len(idx_cubeinbase) - 1, -1, -1):
#         del test[idx_cubeinbase[i]]
#
# print(test)

# dict1 = {"a": 1, "b": 2}
# dict2 = {"c": 3, "d": 4}
#
# merged_dict = {**dict1, **dict2}
# print(merged_dict)

# for i in range(3):
#     print(i)

"""
from filterpy.kalman import KalmanFilter
import numpy as np

# 定义Kalman滤波器
kf = KalmanFilter(dim_x=4, dim_z=2)  # 状态向量为4维，观测向量为2维

# 定义状态转移矩阵，表示系统从t-1时刻到t时刻的状态转移
kf.F = np.array([[1, 0, 1, 0],
                 [0, 1, 0, 1],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

# 定义观测矩阵，表示观测向量与状态向量之间的映射关系
kf.H = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0]])

# 定义过程噪声协方差矩阵，表示状态转移的噪声
kf.Q = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

# 定义观测噪声协方差矩阵，表示观测的噪声
kf.R = np.array([[1, 0],
                 [0, 1]])

# 初始化滤波器的状态向量
kf.x = np.array([0, 0, 0, 0])

# 初始化滤波器的状态协方差矩阵
kf.P = np.eye(4)

# 对每个接收到的位置信息进行滤波
def filter_position(tag_position):
    # 预测状态
    kf.predict()

    # 更新观测
    kf.update(tag_position)

    # 获取估计的位置信息
    filtered_position = kf.x[:2]

    return filtered_position

# 示例使用
# 假设tag_position为apriltag返回的位置信息，形如[x, y]
tag_position = np.array([[0.1,0.5],[0.12,0.55],[0.14,0.6],[100,-50],[0.16,0.65],[0.18,0.7]])

for i in range(len(tag_position)):
    # 使用Kalman滤波器进行位置估计
    filtered_position = filter_position(tag_position[i])

    print("Filtered Position:", filtered_position)
"""

test = [0,1]
idx = [0]
for i in range(len(idx) - 1, -1, -1):
    del test[idx[i]]
print(test)