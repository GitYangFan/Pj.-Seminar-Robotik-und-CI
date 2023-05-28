# This file is only for testing single code paragraph
import cv2
import imutils
import numpy as np
from imutils import paths
# import distance_angle_Object

"""
------------------- Threshold Tuning for canny --------------------
"""
img = cv2.imread('./data/detection_real/1_cut.jpg',0)   # parameter 0 means reading in black-and-white
lowThreshold = 0
max_lowThreshold = 100

maxThreshold = 100
max_maxThreshold = 200
kernel_size = 3

def canny_low_threshold(intial):
    blur = cv2.GaussianBlur(img, (3, 3), 0)
    canny = cv2.Canny(blur, intial,maxThreshold)  # x是最小阈值,y是最大阈值
    cv2.imshow('canny', canny)

def canny_max_threshold(intial):
    blur = cv2.GaussianBlur(img, (3, 3), 0)
    canny = cv2.Canny(blur, lowThreshold,intial)  # x是最小阈值,y是最大阈值
    cv2.imshow('canny', canny)

cv2.namedWindow('canny', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
cv2.createTrackbar('Min threshold', 'canny', lowThreshold, max_lowThreshold, canny_low_threshold)
cv2.createTrackbar('Max threshold', 'canny', maxThreshold, max_maxThreshold, canny_max_threshold)
canny_low_threshold(0)

if cv2.waitKey(0) == 27:    # 27是ESC键值
    cv2.destroyAllWindows()


"""
if __name__ == "__main__":

    # 读取图像
    img = cv2.imread('./data/detection_real/1.jpg')

    # 转换为灰度图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dilated_img = cv2.dilate(gray, np.ones((7, 7), np.uint8))
    bg_img = cv2.medianBlur(dilated_img, 21)
    diff_img = 255 - cv2.absdiff(gray, bg_img)
    norm_img = cv2.normalize(diff_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    cv2.imshow('diff_img', diff_img)
    cv2.imshow('norm_img', norm_img)

    edges = cv2.Canny(norm_img, 10, 100, apertureSize=3)
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if len(contour) >= 200:
            cv2.drawContours(img, contour, -1, (0, 255, 0), 2)
    cv2.imshow('contours', img)

    for contour in contours:
        if len(contour) >= 50:
            ellipses = cv2.fitEllipse(contour)
            cv2.ellipse(img, ellipses, (0, 255, 0), 2)
    # cv2.imshow("ellipse", img)

    # 计算直方图
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    cv2.imshow('gray', gray)
    # 找到直方图的最大值和最小值
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(hist)

    # 创建一个新的空白图像，大小与原始图像相同
    dst = np.zeros(img.shape, dtype=np.uint8)

    # 计算对比度和亮度的增益和偏置值
    alpha = 255 / (maxVal - minVal)
    beta = -minVal * alpha
    beta = 3

    # 应用增益和偏置值来调整对比度和亮度
    cv2.addWeighted(img, alpha, dst, 0, beta, dst)

    # 显示结果
    cv2.imshow('result', dst)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
"""