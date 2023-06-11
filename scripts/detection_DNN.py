from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt",
                input_blob="input_0", output_cvg="scores", output_bbox="boxes",
                threshold=0.5)
camera = videoSource("csi://0")  # '/dev/video0' for V4L2
display = videoOutput("display://0")  # 'my_video.mp4' for file

while display.IsStreaming():
    img = camera.Capture()

    if img is None:  # capture timeout
        continue

    detections = net.Detect(img)
    print(detections)

    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))