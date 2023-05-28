# color_correction.py
#  用法
# python color_correction.py --reference reference.jpg --input examples/03.jpg
# 导入相关库
from imutils.perspective import four_point_transform
from skimage import exposure
import numpy as np
import argparse
import imutils
import cv2
import sys

def find_color_card(image):
    # 加载ArUCo字典，获取ArUCo参数并检测输入图像中的标记
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    # 尝试提取颜色校正卡的坐标
    try:
        # 否则，这意味着四个ArUCo标记已经被发现，因此继续扁平化ArUCo id列表
        ids = ids.flatten()
        # 提取左上角的标记
        i = np.squeeze(np.where(ids == 923))
        topLeft = np.squeeze(corners[i])[0]
        # 提取右上角的标记
        i = np.squeeze(np.where(ids == 1001))
        topRight = np.squeeze(corners[i])[1]
        # 提取右下角的标记
        i = np.squeeze(np.where(ids == 241))
        bottomRight = np.squeeze(corners[i])[2]
        # 提取左下角的标记
        i = np.squeeze(np.where(ids == 1007))
        bottomLeft = np.squeeze(corners[i])[3]
    # 如果校色卡找不到了
    except:
        return None
    # 构建参考点列表并应用透视变换以获得配色卡的自上而下的鸟瞰图
    cardCoords = np.array([topLeft, topRight, bottomRight, bottomLeft])
    card = four_point_transform(image, cardCoords)
    # 将颜色匹配卡返回给调用函数
    return card


# 构造参数解析器并解析参数
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--reference", required=True, help="Path to the input reference image")
ap.add_argument("-i", "--input", required=True, help="Path to the input image to apply color correction to")
args = vars(ap.parse_args())

# 从磁盘加载参考图像和输入图像
print("[INFO] Loading images...")
ref = cv2.imread(args["reference"])
image = cv2.imread(args["input"])

# 调整参考和输入图像的大小
ref = imutils.resize(ref, width=600)
image = imutils.resize(image, width=600)

# 在屏幕上显示参考和输入图像
cv2.imshow("Reference", ref)
cv2.imshow("Input", image)

# 找出每张图片中的配色卡
print("[INFO] Finding color matching cards...")
refCard = find_color_card(ref)
imageCard = find_color_card(image)

# 如果在参考或输入图像中都没有找到配色卡，请退出程序
if refCard is None or imageCard is None:
    print("[INFO] Could not find color matching cards in both images! Exiting...")
    sys.exit(0)

# 分别在参考图像和输入图像中显示颜色匹配卡
cv2.imshow("Reference Color Card", refCard)
cv2.imshow("Input Color Card", imageCard)

# 将参考图像中的颜色匹配卡的直方图匹配应用到输入图像中的颜色匹配卡
print("[INFO] Matching images...")
imageCard = exposure.match_histograms(imageCard, refCard, multichannel=True)

# 直方图匹配后显示输入的颜色匹配卡
cv2.imshow("Input Color Card After Matching", imageCard)
cv2.waitKey(0)
