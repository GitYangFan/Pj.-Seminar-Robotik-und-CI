import numpy as np
import cv2
import collections
import math

color_dict = collections.defaultdict(list)
lower_red1 = np.array([0, 43, 46])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([156, 43, 46])
upper_red2 = np.array([180, 255, 255])
red_list = []
red_list.append(lower_red1)
red_list.append(upper_red1)
red_list.append(lower_red2)
red_list.append(upper_red2)
color_dict['red'] = red_list

lower_green = np.array([35, 43, 46])
upper_green = np.array([77, 255, 255])
green_list = []
green_list.append(lower_green)
green_list.append(upper_green)
color_dict['green'] = green_list

lower_blue = np.array([100, 43, 46])
upper_blue = np.array([124, 255, 255])
blue_list = []
blue_list.append(lower_blue)
blue_list.append(upper_blue)
color_dict['blue'] = blue_list

lower_yellow = np.array([26, 43, 46])
upper_yellow = np.array([34, 255, 255])
yellow_list = []
yellow_list.append(lower_yellow)
yellow_list.append(upper_yellow)
color_dict['yellow'] = yellow_list

lower_purple = np.array([125, 43, 46])
upper_purple = np.array([155, 255, 255])
purple_list = []
purple_list.append(lower_purple)
purple_list.append(upper_purple)
color_dict['purple'] = purple_list

lower_orange = np.array([11, 43, 46])
upper_orange = np.array([25, 255, 255])
orange_list = []
orange_list.append(lower_orange)
orange_list.append(upper_orange)
color_dict['orange'] = orange_list

class Cube:
    """
    class of cube

    Attributes:
        contour: (approxmated) contour of the cube in 2D
        mc: mass center of the cube
        color: color of the cube
    """
    def __init__(self, approx, color):
        self.approx = approx
        # self.mc = mc
        self.color = color

class Ball:
    """
    class of ball

    Attributes:
        center: center of the ball in 2D
        radius: radius of the ball in 2D
    """

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        

def cvImshow(window_name, image):
    """
    Easy function to show image

    Args:
        window_name: name of the window
        image: path of the image
    """
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def getMassCenter(contours):
    """
    Function to get the mass center of contours

    Args:
        contours (list): list of contour

    Returns:
        mc (list): list of mass centers
    """

    # Get the moments
    mu = [None]*len(contours)
    for i in range(len(contours)):
        mu[i] = cv2.moments(contours[i])

    # Get the mass centers
    mc = [None]*len(contours)
    for i in range(len(contours)):
        # add 1e-6 to avoid division by zero
        mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-6), mu[i]['m01'] / (mu[i]['m00'] + 1e-6)) # type: ignore

    return np.uint16(np.around(mc)) # type: ignore


def filterRepeatedContours(contours, mc):
    """
    Function to remove the repeated contours and its mass center

    Args:
        contours (list): list of contours need to filter
        mc (list): list of mass centers need to filter

    Returns:
        contours (list): list of contours after filtering
        mc (list): list of mass centers after filtering
    """

    # use distance between two mass centers, difference of areas and difference of shape to filter
    is_valid = np.ones(len(contours), dtype=bool)
    area = [cv2.contourArea(cnt) for cnt in contours]
    for i in range(len(contours)):
        if is_valid[i]:
            area_i = area[i]
            for j in range(i + 1, len(contours)):
                vec = mc[i] - mc[j]
                distance = np.linalg.norm(vec)
                area_j = area[j]
                area_diff = math.fabs(area_i-area_j)
                match = cv2.matchShapes(contours[i], contours[j], 1, 0.0)
                if distance < 30 and area_diff < 2400 and match < 0.1: # to adjust
                    is_valid[j] = False
    contours = [contours[i] for i in range(len(contours)) if is_valid[i]]
    mc = [mc[i] for i in range(len(mc)) if is_valid[i]]

    return contours, mc

def increaseBrightness(img, beta=30):
    """
    Function to increase brightness of the image

    Args:
        img: the image in BGR color
        beta: the value of the brightness to increase

    Return:
        output_img: the new image in BGR color
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - beta
    v[v > lim] = 255
    v[v <= lim] += beta

    hsv = cv2.merge((h, s, v))
    output_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


    return output_img

def decreaseBrightness(img, beta=30):
    """
    Function to decrease brightness of the image

    Args:
        img: the image in BGR color
        beta: the value of the brightness to decrease

    Return:
        output_img: the new image in BGR color
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[v >= beta] -= beta
    v[v < beta] = 0

    hsv = cv2.merge((h, s, v))
    output_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    return output_img

def colorBalance(src, mode):
    """
    Function to adjust glo3.6bally the intensities of the color

    Args:
        src: the image need to process
        mode: the mode of the process

    Return:
        the image after processing
    """

    output_img = src.copy()
    b, g, r = cv2.split(src)
    h, w, c = src.shape
    if mode == 0:
        sum_ = np.double() + b + g + r
        hists, _ = np.histogram(sum_.flatten(), 766, [0, 766])
        Y = 765
        num, key = 0, 0
        ratio = 0.05
        while Y >= 0:
            num += hists[Y]
            if num > h * w * ratio / 100:
                key = Y
                break
            Y = Y - 1

        sumkey = np.where(sum_ >= key)
        sum_b, sum_g, sum_r = np.sum(b[sumkey]), np.sum(g[sumkey]), np.sum(r[sumkey])
        times = len(sumkey[0])
        avg_b, avg_g, avg_r = sum_b / times, sum_g / times, sum_r / times

        maxvalue = float(np.max(output_img))
        output_img[:, :, 0] = output_img[:, :, 0] * maxvalue / int(avg_b)
        output_img[:, :, 1] = output_img[:, :, 1] * maxvalue / int(avg_g)
        output_img[:, :, 2] = output_img[:, :, 2] * maxvalue / int(avg_r)
    elif mode == 1:
        I_b_2, I_r_2 = np.double(b) ** 2, np.double(r) ** 2
        sum_I_b_2, sum_I_r_2 = np.sum(I_b_2), np.sum(I_r_2)
        sum_I_b, sum_I_g, sum_I_r = np.sum(b), np.sum(g), np.sum(r)
        max_I_b, max_I_g, max_I_r = np.max(b), np.max(g), np.max(r)
        max_I_b_2, max_I_r_2 = np.max(I_b_2), np.max(I_r_2)
        [u_b, v_b] = np.matmul(np.linalg.inv([[sum_I_b_2, sum_I_b], [max_I_b_2, max_I_b]]), [sum_I_g, max_I_g])
        [u_r, v_r] = np.matmul(np.linalg.inv([[sum_I_r_2, sum_I_r], [max_I_r_2, max_I_r]]), [sum_I_g, max_I_g])
        b0 = np.uint8(u_b * (np.double(b) ** 2) + v_b * b)
        r0 = np.uint8(u_r * (np.double(r) ** 2) + v_r * r)
        output_img = cv2.merge([b0, g, r0])

    elif mode == 2:
        def con_num(x):
            if x > 0:
                return 1
            if x < 0:
                return -1
            if x == 0:
                return 0

        yuv_img = cv2.cvtColor(src, cv2.COLOR_BGR2YCrCb)
        # YUV空间
        (y, u, v) = cv2.split(yuv_img)
        max_y = np.max(y.flatten())
        sum_u, sum_v = np.sum(u), np.sum(v)
        avl_u, avl_v = sum_u / (h * w), sum_v / (h * w)
        du, dv = np.sum(np.abs(u - avl_u)), np.sum(np.abs(v - avl_v))
        avl_du, avl_dv = du / (h * w), dv / (h * w)
        radio = 0.5 # 如果该值过大过小，色温向两极端发展

        valuekey = np.where((np.abs(u - (avl_u + avl_du * con_num(avl_u))) < radio * avl_du)
                             | (np.abs(v - (avl_v + avl_dv * con_num(avl_v))) < radio * avl_dv))
        num_y, yhistogram = np.zeros((h, w)), np.zeros(256)
        num_y[valuekey] = np.uint8(y[valuekey])
        yhistogram = np.bincount(np.uint8(num_y[valuekey].flatten()), minlength=256)
        ysum = len(valuekey[0])
        Y = 255
        num, key = 0, 0
        while Y >= 0:
            num += yhistogram[Y]
            if num > 0.1 * ysum:  # 取前10%的亮点为计算值，如果该值过大易过曝光，该值过小调整幅度小
                key = Y
                break
            Y = Y - 1

        sumkey = np.where(num_y > key)
        sum_b, sum_g, sum_r = np.sum(b[sumkey]), np.sum(g[sumkey]), np.sum(r[sumkey])
        num_rgb = len(sumkey[0])

        b0 = np.double(b) * int(max_y) / (sum_b / num_rgb)
        g0 = np.double(g) * int(max_y) / (sum_g / num_rgb)
        r0 = np.double(r) * int(max_y) / (sum_r / num_rgb)

        output_img = cv2.merge([b0, g0, r0])
    else:
        b_avg, g_avg, r_avg = cv2.mean(b)[0], cv2.mean(g)[0], cv2.mean(r)[0]
        k = (b_avg + g_avg + r_avg) / 3
        kb, kg, kr = k / b_avg, k / g_avg, k / r_avg
        b = cv2.addWeighted(src1=b, alpha=kb, src2=0, beta=0, gamma=0)
        g = cv2.addWeighted(src1=g, alpha=kg, src2=0, beta=0, gamma=0)
        r = cv2.addWeighted(src1=r, alpha=kr, src2=0, beta=0, gamma=0)
        output_img = cv2.merge([b, g, r])

    output_img = np.uint8(np.clip(output_img, 0, 255))
    return output_img


def cubeDetection(blurred, color_dict):
    """
    Function to detect cube with different color

    Args:
        blurred: the blurred source image
        color_dict: the dictionary of different color

    Return:
        cubes: list of cubes
    """
    cubes = []
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv_morph = cv2.dilate(hsv, None, iterations=2)
    hsv_morph = cv2.erode(hsv, None, iterations=2)

    for color in color_dict:
        print(color)
        mask = cv2.inRange(hsv_morph, color_dict[color][0], color_dict[color][1])
        if color == "red":
            mask += cv2.inRange(hsv_morph, color_dict[color][2], color_dict[color][3])
            # mask = cv2.bitwise_or(mask, cv2.inRange(hsv_morph, color[color][2], color_dict[color][3]))
        cvImshow(window_name, mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mc = getMassCenter(contours)
        contours, mc = filterRepeatedContours(contours, mc)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 300 or area > 0.1 * blurred.shape[0] * blurred.shape[1]:
                continue
            epsilon = 0.02*cv2.arcLength(contours[i], True)
            approx = cv2.approxPolyDP(contours[i], epsilon,True)
            if cv2.isContourConvex(approx):
                if len(approx) == 4 or len(approx) == 6:
                    contour_area = cv2.contourArea(contours[i])
                    approx_area = cv2.contourArea(approx)
                    if math.fabs(contour_area - approx_area) / contour_area < 0.05: # to adjust
                        cube = Cube(approx, color)
                        cubes.append(cube)

    return cubes

def ballDetection(blurred):
    """
    Function to detect ball, don't care the color of the ball

    Args:
        blurred: the blurred source image

    Return:
        balls: list of ball
    """

    balls = []
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    # gray = cv2.equalizeHist(gray)
    gray_morph = cv2.dilate(gray, None, iterations=2)
    gray_morph = cv2.erode(gray, None, iterations=2)
    cvImshow(window_name, gray_morph)

    # gray: Input image (grayscale).
    #  circles: A vector that stores sets of 3 values: *c, Yc, r for each detected circle.
    #  HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
    #  dp = 1: The inverse ratio of resolution.
    #  min_dist = gray.rows/8: Minimum distance between detected centers.
    #  param_ 1 = 200: Upper threshold for the internal Canny edge detector.
    #  param_2 = 100*: Threshold for center detection.
    #  min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
    #  max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.
    circles = cv2.HoughCircles(gray_morph, cv2.HOUGH_GRADIENT, 1, gray.shape[0] / 8,
                                param1=300, param2=60,
                                minRadius=5, maxRadius=np.uint16(math.sqrt(gray.shape[0]**2 + gray.shape[1]**2) / 8))
    # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, gray.shape[0] / 8,
    #                             param1=240, param2=35,
    #                             minRadius=8, maxRadius=500)

    if circles is not None:
        for circle in circles[0, :]:
            center = np.uint16(np.around((circle[0], circle[1])))
            ball = Ball(center, np.uint16(circle[2]))
            balls.append(ball)

    return balls


def drawCubes(src, cubes):
    """
    Function to draw cubes in src

    Args:
        src: the source image
        cubes (list): list of cube
    """
    for cube in cubes:
        cv2.drawContours(src, [cube.approx], -1, (255, 255,0), 3)
        cv2.putText(src, str(cube.color), (cube.approx[0][0][0], cube.approx[0][0][1]), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 180, 255), 2)
        # cv2.circle(src, cube.mc, 1, (255, 255, 0), 3) #type: ignore

def drawBalls(src, balls):
    """
    Function to draw balls in src

    Args:
        src: the source image
        balls (list): list of balls
    """
    for ball in balls:
        cv2.circle(src, ball.center, ball.radius, (255, 0, 255), 3)
        cv2.circle(src, ball.center, 1, (255, 5, 255), 3)


# test
path = "./img/4.jpg"
window_name = "image"

src = cv2.imread(path)
src = src[int(src.shape[0] / 2.5) : src.shape[0], int(src.shape[1] / 6) : int(src.shape[1] / 6 * 5)]
# src = src[int(src.shape[0] / 2.7) : src.shape[0], 0 : src.shape[1]]
src = colorBalance(src, 1)

# b, g, r = cv2.split(src)
# r = cv2.addWeighted(r, 0.5, g, 0.5, 0)
# b = cv2.addWeighted(b, 0.5, g, 0.5, 0)
# src = cv2.merge((b, g, r))


# lab
# lab = cv2.cvtColor(src, cv2.COLOR_BGR2LAB)
# l_channel, a_channel, b_channel = cv2.split(lab)
# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(5, 5))
# l_channel_eq = clahe.apply(l_channel)
# lab_eq = cv2.merge((l_channel_eq, a_channel, b_channel))
# src = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

cvImshow(window_name, src)
blurred = cv2.GaussianBlur(src, (5, 5), 0)

cubes = cubeDetection(increaseBrightness(blurred, 30), color_dict)
# cubes = cubeDetection(blurred, color_dict)
# balls = ballDetection(decreaseBrightness(blurred, 30))
balls = ballDetection(blurred)

print("number of cubes: ", len(cubes))
for cube in cubes:
    print("approx: \n", cube.approx)
    print("color: ", cube.color, '\n')
print("number of balls: ", len(balls))
for ball in balls:
    print("center: ", ball.center)
    print("radius: ", ball.radius, '\n')

drawCubes(src, cubes)
drawBalls(src, balls)

cvImshow(window_name, src)
