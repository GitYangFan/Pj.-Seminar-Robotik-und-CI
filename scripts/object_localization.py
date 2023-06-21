import rospy
import numpy as np
from detection_results import get_detection
from apriltag import get_apriltag

# The function to calculate the focallength
def get_focal_length():
    # initialize the known distance from the camera to the known object, unit in mm.
    distance_real = 0.15  # unit: m
    length_real = 0.03   # unit: m
    length_pixel = 168 # unit: pixel
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

def find_object():
    camera_frame = [1080, 720]
    focal_length = get_focal_length()
    print('focalLength:', focal_length)
    object_name, object_score, object_center, object_size = get_detection()     # get the detection results from DNN model
    length = len(object_name)
    object_distance = [None] * length
    object_angle = [None] * length

    for i in range(length):
        if object_name[i] == 'ball':
            object_distance[i] = distance_to_camera(0.028, focal_length,object_size[i][0])  # the real size of ball is 0.028m
        else:
            object_distance[i] = distance_to_camera(0.03, focal_length, object_size[i][0])   # the real size of cube is 0.03m
        object_angle[i] = angle_to_camera(focal_length, object_center[i], camera_frame)
        #print('detected:',object_name[i], 'possibility:',object_score[i], 'distance:', object_distance[i], 'angle:', object_angle[i])
        #print('------------split---------------')
    return object_name, object_score, object_distance, object_angle

def get_object_position():
    position_jetbot, orientation_jetbot = get_apriltag()
    print('position_jetbot:', position_jetbot, 'orientation_jetbot', orientation_jetbot)
    object_name, object_score, object_distance, object_angle = find_object()
    length = len(object_name)
    object_position = [None] * length
    for i in range(length):
        object_position[i] = position_jetbot[0:2] + (object_distance[i] * np.cos(object_angle[i][1])) * (orientation_jetbot[2] + object_angle[i][0])
        print('detected:',object_name[i], 'possibility:',object_score[i], 'position:', object_position[i], 'distance:', object_distance[i], 'angle:', object_angle[i])
        print('------------split---------------')
    return object_name, object_score, object_position

"""
test 2
"""
rospy.init_node('object_localization')
# object_name, object_score, object_distance, object_angle = find_object()
object_name, object_score, object_position = get_object_position()
# print('detected:',object_name, 'possibility:',object_score, 'object position:', object_position)
# print('distance list:', object_distance, 'angle list:', object_angle)

"""

test


focalLength = get_focal_length()
print(focalLength)

for imagePath in sorted(paths.list_images("./data/distance")):
    # load the image, find the marker in the image, then compute the
    # distance to the marker from the camera
    image = cv2.imread(imagePath)
    marker = find_marker(image)
    distance = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

    # angle to the marker from the camera
    angle = angle_to_camera(focalLength, [330, 1406], [4032, 1872])
    print(angle[0])
    print(angle[1])
    distance_test = distance_to_camera(30, focalLength, 321)
    print(distance_test)

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
    
"""