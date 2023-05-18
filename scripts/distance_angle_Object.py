import cv2
import imutils
import numpy as np
from imutils import paths

KNOWN_WIDTH = 6.4   # unit: cm

# The function to calculate the focallength
def get_focal_length():
    # initialize the known distance from the camera to the known object, unit in mm.
    distance_real = 50  # unit: cm
    length_real = 6.4   # unit: cm
    length_pixel = 422 # unit: pixel
    # formula of calculating the focal length
    focal_length = (length_pixel * distance_real) / length_real
    return focal_length

# The function to calculate the distance from object to camera
def distance_to_camera(height_real, focal_length, height_pixel):
    """
    Parameters
    ------------
    height_real : the real height(or radius) of the object
    focal_length : the focal length of the camera
    height_pixel : the height in the image, the unit should be pixel

    Returns
    ------------
    distance : the distance from object to camera
    """
    distance = (height_real * focal_length) / height_pixel
    return distance

# The function to calculate the angle in camera coordinate
def angle_to_camera(focal_length, position_pixel = [0, 0], camera_frame = [1080, 720]):
    """
    Parameters
    ------------
    focal_length : the focal length of the camera
    position_pixel : the postion(x,y) in the image coordinate system, The origin is the upper left corner
    camera_frame : the image width and heigth, the default value is (1080x720)

    Returns
    ------------
    angle_horizon : the deflection angle in the horizon plane of the camera coordinate, positive value means left
    angle_vertical : the deflection angle in the vertical plane of the camera coordinate, positive value means upon
    """
    angle_horizon = np.arctan((camera_frame[0]/2 - position_pixel[0])/focal_length)
    angle_vertical = np.arctan((camera_frame[1]/2 - position_pixel[1])/focal_length)
    return angle_horizon, angle_vertical


# template function to find the marker, will be removed after the detector finished
def find_marker(image):
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    # the contour of paper is not closed, so apply close operation(dilate and erode)
    kernel = np.ones((3, 3), np.uint8)
    close = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(close.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key = cv2.contourArea)

    # compute the bounding box of the paper region and return it
    return cv2.minAreaRect(c)

"""

test

"""
focalLength = get_focal_length()
print(focalLength)

for imagePath in sorted(paths.list_images("./data/distance")):
    # load the image, find the marker in the image, then compute the
    # distance to the marker from the camera
    image = cv2.imread(imagePath)
    marker = find_marker(image)
    distance = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

    # angle to the marker from the camera
    angle = angle_to_camera(focalLength, [1162, 1326], [1872, 4032])
    print(angle[0])
    print(angle[1])

    # draw a bounding box around the image and display it
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(image, [box], -1, (0, 255, 0), 10)
    cv2.putText(image, "%.2fcm" % distance,
        (image.shape[1] - 350, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
        2.0, (0, 255, 0), 3)
    cv2.namedWindow(imagePath.split('/')[-1], 0)
    cv2.imshow(imagePath.split('/')[-1], image)

if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()