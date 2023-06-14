import numpy as np
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from image_capture import get_image


# initialization of object list
def get_detection():
    while display.IsStreaming():
        # img = camera.Capture()  # get the image from the csi://0
        img = get_image()     # get the image from the topic: /camera/image_rect_color

        if img is None:  # capture timeout
            continue

        detections = net.Detect(img)
        number = len(detections)
        object_name = [None] * number
        object_center = [None] * number
        object_size = [[0,0]] * number
        idx = 0
        # print(detections)
        for detection in detections:
            object_name[idx] = net.GetClassDesc(detection.ClassID)
            object_center[idx] = detection.Center
            object_size[idx] = [detection.Width, detection.Height]
            print('detected:', object_name[idx], 'center:', object_center[idx], 'size:', object_size[idx])
            idx += 1

        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

        return object_name, object_center, object_size



"""
-------------------- test ----------------------------
"""

net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt",
                input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                threshold=0.5)
camera = videoSource("csi://0")  # '/dev/video0' for V4L2
display = videoOutput("display://0")  # 'my_video.mp4' for file
object_name, object_center, object_size = get_detection()

