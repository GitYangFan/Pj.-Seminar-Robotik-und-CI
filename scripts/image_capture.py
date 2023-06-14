import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import jetson_utils

img = None

def image_callback(msg):
    global img
    bridge = CvBridge()
    img_cv = bridge.imgmsg_to_cv2(msg, "bgr8")  # convert 'Image' to 'numpy' array
    img = jetson_utils.cudaFromNumpy(img_cv)    # convert 'numpy' array to cuda img for using DNN model

def get_image():
    rospy.init_node('image_capture')
    rospy.Subscriber('/camera/image_rect_color', Image, image_callback)
    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if img is not None:
            print('Image captured successfully.')
            return img
        rospy.sleep(0.1)


"""
-------------------- test ----------------------------
"""
