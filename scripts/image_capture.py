import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import jetson.utils

img = None

def image_callback(msg):
    global img
    bridge = CvBridge()
    try:
        img_cv = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = jetson.utils.cudaFromNumpy(img_cv)
    except Exception as e:
        rospy.logerr(e)

def get_image():
    rospy.init_node('image_capture')
    rospy.Subscriber('image_rect_color', Image, image_callback, queue_size=1)
    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if img is not None:
            print('Image captured successfully.')
            return img
        rospy.sleep(0.1)


"""
-------------------- test ----------------------------
"""
if __name__ == '__main__':
    img = get_image()
    print(img)