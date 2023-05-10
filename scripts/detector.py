import numpy as np
import cv2
import collections
import math

color_dict = collections.defaultdict(list)
lower_red1 = np.array([0, 43, 46])
upper_red1 = np.array([10, 255, 255])
red1_list = []
red1_list.append(lower_red1)
red1_list.append(upper_red1)
color_dict['red1'] = red1_list

lower_red2 = np.array([156, 43, 46])
upper_red2 = np.array([180, 255, 255])
red2_list = []
red2_list.append(lower_red2)
red2_list.append(upper_red2)
color_dict['red2'] = red2_list

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
    def __init__(self, contour, mc, color):
        self.contour = contour
        self.mc = mc
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
    hsv_erode = cv2.erode(hsv, None, iterations=2)

    for color in color_dict:
        mask = cv2.inRange(hsv_erode, color_dict[color][0], color_dict[color][1])
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mc = getMassCenter(contours)
        contours, mc = filterRepeatedContours(contours, mc)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 100 or area > 0.95 * blurred.shape[0] * blurred.shape[1]:
                continue
            epsilon = 0.01*cv2.arcLength(contours[i], True)
            approx = cv2.approxPolyDP(contours[i], epsilon,True)
            if len(approx) == 4 or len(approx) == 6:
                cube = Cube(contours[i], mc[i], color)
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

    # gray: Input image (grayscale).
    #  circles: A vector that stores sets of 3 values: *c, Yc, r for each detected circle.
    #  HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
    #  dp = 1: The inverse ratio of resolution.
    #  min_dist = gray.rows/8: Minimum distance between detected centers.
    #  param_ 1 = 200: Upper threshold for the internal Canny edge detector.
    #  param_2 = 100*: Threshold for center detection.
    #  min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
    #  max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, gray.shape[0] / 8,
                                param1=200, param2=50,
                                minRadius=5, maxRadius=300)

    if circles != None:
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
        cv2.drawContours(src, [cube.contour], -1, (255, 255,0), 3)
        cv2.circle(src, cube.mc, 1, (255, 255, 0), 3) #type: ignore

def drawBalls(src, balls):
    """
    Function to draw balls in src

    Args:
        src: the source image
        balls (list): list of balls
    """
    for ball in balls:
        cv2.circle(src, ball.center, ball.radius, (255, 0, 255), 3)
        cv2.circle(src, ball.center, 1, (255, 0, 255), 3)


# test
path = "./colorCube/blue_cube0.png"
window_name = "image"

src = cv2.imread(path);
blurred = cv2.GaussianBlur(src, (5, 5), 0)

cubes = cubeDetection(blurred, color_dict)
balls = ballDetection(blurred)

print("number of cube: ", len(cubes))
for cube in cubes:
    print(cube.mc, cube.color)
print("number of ball: ", len(balls))
for ball in balls:
    print(ball.center)

drawCubes(src, cubes)
drawBalls(src, balls)

cvImshow(window_name, src)
