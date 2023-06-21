from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from image_capture import get_image

net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt",
                input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                threshold=0.5)
camera = videoSource("csi://0")  # '/dev/video0' for V4L2
display = videoOutput("display://0")  # 'my_video.mp4' for file

# initialization of object list

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
        print('detected:', object_name, 'center:', object_center, 'size:', object_size)


    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))