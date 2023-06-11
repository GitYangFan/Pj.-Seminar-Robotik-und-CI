import rospy
from sensor_msgs.msg import Image


def image_callback(msg):
    global img
    img = msg


def get_image():
    img = None
    rospy.init_node('image_capture')
    rospy.Subscriber('image_rect_color', Image, image_callback, queue_size=1)

    while not rospy.is_shutdown() and img is None:
        rospy.sleep(0.1)
    if img is not None:
        print('Image captured successfully.')
        return img


"""
-------------------- test ----------------------------
"""
if __name__ == '__main__':
    img = get_image()
    print(img)
    rospy.spin()