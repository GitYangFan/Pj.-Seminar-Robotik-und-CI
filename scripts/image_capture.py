import rospy
from sensor_msgs.msg import Image
import numpy as np
#from cv_bridge import CvBridge
import jetson_utils

img = None
#bridge = CvBridge()

def image_callback(msg):
    global img
    # img_cv = bridge.imgmsg_to_cv2(msg, "bgr8")
    img_cv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    img = jetson_utils.cudaFromNumpy(img_cv)


def get_image():
    rospy.init_node('image_capture')
    rospy.Subscriber('/camera/image_rect_color', Image, image_callback)
    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if img is not None:
            print('Image captured successfully.')
            return img
        else:
            print('failed to capture image.')
        rospy.sleep(0.1)


"""
-------------------- test ----------------------------
"""
if __name__ == '__main__':
    image = get_image()
    print(image)
