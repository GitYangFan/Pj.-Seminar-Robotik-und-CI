from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from image_capture import get_image
import math
from basic_move import turn_clockwise
from basic_move import turn_counterclockwise
from basic_move import ahead_distance
import random

net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt",
                input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                threshold=0.5)
camera = videoSource("csi://0")  # '/dev/video0' for V4L2
display = videoOutput("display://0")  # 'my_video.mp4' for file

# initialization of object list
def calculate_distance(object_center, focal, object_width):

    distance = (object_width * focal) / object_center[0]

    return distance
focal = 100
object_width= 5
def operation1():
    turn_clockwise(jetbot_motor)
def operation2():
    turn_counterclockwise(jetbot_motor)

while display.IsStreaming():
    # img = camera.Capture()
    img = get_image()

    if img is None:  # capture timeout
        continue

    detections = net.Detect(img)
    # print(detections)
    for detection in detections:
        object_name = net.GetClassDesc(detection.ClassID)
        object_center = detection.Center
        object_size = [detection.Width, detection.Height]
        distance = calculate_distance(object_center, focal, object_width)
        if "ball" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_red" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_orange" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_yellow" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_green" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_purple" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        elif "cube_blue" in object_name and distance < 20:
            operations = [operation1, operation2]
            selected_operation = random.choice(operations)
            selected_operation()
        else:
            ahead_distance(jetbot_motor, distance)
    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))








