import rospy
from vision_msgs.msg import Detection2DArray

detections = []
class_name = ['BACKGROUND', 'ball', 'cube_red', 'cube_orange', 'cube_yellow', 'cube_green', 'cube_purple', 'cube_blue']

def detection_callback(data):
    global detections
    if data.detections != []:
        detections = data.detections

def get_detection():
    # rospy.init_node('detection_listener')
    rospy.Subscriber('/detectnet/detections', Detection2DArray, detection_callback)

    # Loop waiting to receive data
    while not rospy.is_shutdown():
        if detections !=[]:
            # print(detections)
            # print('-----------split---------------')
            length = len(detections)
            object_name = [None] * length
            object_score = [None] * length
            object_center = [None] * length
            object_size = [None] * length
            for i in range(length):
                detection = detections[i]
                object_name[i] = class_name[detection.results[0].id]
                object_score[i] = detection.results[0].score
                object_center[i] = [detection.bbox.center.x, detection.bbox.center.y]
                object_size[i] = [detection.bbox.size_x, detection.bbox.size_y]
                # print('detected: ',object_name[i], 'score: ', object_score[i], 'center: ', object_center[i], 'size', object_size[i])
                # print('next one')
            # print('list:', object_name)
            # print('-----------split---------------')
            return object_name, object_score, object_center, object_size
        rospy.sleep(0.1)


"""
-------------------- test ----------------------------
"""



# object_name, object_score, object_center, object_size = get_detection()
# print('detected:', object_name[0], 'score:', object_score[0], 'center:', object_center[0], 'size:', object_size[0])
