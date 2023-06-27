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

import numpy as np
import tf

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

test = []
length = len(test)
distance = [None]*length
print(length)
print(distance)
# print(range(len(test)))
for i in range(length):
    print(i)