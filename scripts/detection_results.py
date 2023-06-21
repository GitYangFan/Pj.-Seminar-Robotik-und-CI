import rospy
from vision_msgs.msg import Detection2DArray

detection = []

def detection_callback(data):
    global detection
    if data.detections != []:
        detection = [data.results, data.bbox]

def get_detection():
    rospy.init_node('detection_listener')
    rospy.Subscriber('/tag_detections', Detection2DArray, detection_callback)

    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if detection !=[]:
            print(detection)
            # return object_name, object_center, object_size
        rospy.sleep(0.1)