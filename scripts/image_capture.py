import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 初始化ROS节点
rospy.init_node('image_capture_node')

# 创建一个用于接收图像的变量
img = None

# 回调函数，用于处理接收到的图像数据
def image_callback(msg):
    global img
    # 取消订阅图像话题，因为我们只需要获取一张照片
    rospy.Subscriber('image_rect_color', Image, image_callback, queue_size=1)

# 等待接收到第一张照片
rospy.wait_for_message('image_rect_color', Image)

# 保存图像
if img is not None:
    print('Image captured successfully.')
else:
    print('Failed to capture the image.')

# 关闭ROS节点
rospy.shutdown()