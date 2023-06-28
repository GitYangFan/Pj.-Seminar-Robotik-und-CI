# import math
# import matplotlib.pyplot as plt
#
# def find_point_on_extended_line(A, B, distance):
#     # 计算向量AB的分量表示
#     AB = (B[0] - A[0], B[1] - A[1])
#
#     # 单位化向量AB
#     magnitude = math.sqrt(AB[0]**2 + AB[1]**2)
#     uAB = (AB[0] / magnitude, AB[1] / magnitude)
#
#     # 计算P点的坐标
#     Px = (distance * uAB[0]) + B[0]
#     Py = (distance * uAB[1]) + B[1]
#
#     return (Px, Py)
#
# def plot_extended_line_with_point(A, B, P):
#     # 绘制直线AB及其延长线
#     plt.plot([A[0], B[0]], [A[1], B[1]], 'b-', label='AB')
#     plt.plot([B[0], P[0]], [B[1], P[1]], 'r--', label='Extended Line')
#
#     # 绘制点A，B和点P
#     plt.plot(A[0], A[1], 'ro', label='A')
#     plt.plot(B[0], B[1], 'go', label='B')
#     plt.plot(P[0], P[1], 'bo', label='P')
#
#     # 添加坐标轴标签和图例
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.legend()
#
#     # 设置坐标轴范围
#     plt.xlim(min(A[0], B[0], P[0])-1, max(A[0], B[0], P[0])+1)
#     plt.ylim(min(A[1], B[1], P[1])-1, max(A[1], B[1], P[1])+1)
#
#     # 显示网格
#     plt.grid(True)
#
#     # 显示图形
#     plt.show()
#
# # 测试示例
# A = (-5, 8)
# B = (-3, -6)
# distance = 2
#
# P = find_point_on_extended_line(A, B, distance)
# print("P点的坐标为:", P)
#
# # 绘制直线AB及其延长线，以及点P
# plot_extended_line_with_point(A, B, P)

import matplotlib.pyplot as plt

def move_to_segment(A, B, P):
    # 计算线段AB的长度
    segment_length = ((B[0] - A[0])**2 + (B[1] - A[1])**2)**0.5

    # 计算向量PA和向量AB
    PA = [A[0] - P[0], A[1] - P[1]]
    AB = [B[0] - A[0], B[1] - A[1]]

    # 计算投影长度
    projection_length = (PA[0] * AB[0] + PA[1] * AB[1]) / segment_length

    if projection_length <= 0:
        # 如果投影长度小于等于0，返回点A的坐标
        return A
    elif projection_length >= segment_length:
        # 如果投影长度大于等于线段AB的长度，返回点B的坐标
        return B
    else:
        # 计算投影比例
        t = projection_length / segment_length

        # 计算恰好回到线段AB上的点的坐标
        return (A[0] + t * (B[0] - A[0]), A[1] + t * (B[1] - A[1]))

def plot_segment_movement(A, B, P, result):
    # 绘制线段AB
    plt.plot([A[0], B[0]], [A[1], B[1]], 'b-', label='AB')

    # 绘制点A、B、P和结果点
    plt.plot(A[0], A[1], 'ro', label='A')
    plt.plot(B[0], B[1], 'ro', label='B')
    plt.plot(P[0], P[1], 'go', label='P')
    plt.plot(result[0], result[1], 'bo', label='Result')

    # 添加坐标轴标签和图例
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    # 设置坐标轴范围
    plt.xlim(min(A[0], B[0], P[0], result[0])-1, max(A[0], B[0], P[0], result[0])+1)
    plt.ylim(min(A[1], B[1], P[1], result[1])-1, max(A[1], B[1], P[1], result[1])+1)

    # 显示网格
    plt.grid(True)

    # 显示图形
    plt.show()

# 测试示例
A = (2, 3)
B = (6, 5)
P = (4, 8)

result = move_to_segment(A, B, P)
print("恰好回到线段AB上的点的坐标为:", result)

# 绘制线段AB、点A、B、P和结果点
plot_segment_movement(A, B, P, result)
