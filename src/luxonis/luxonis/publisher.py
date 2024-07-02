import rclpy
import depthai as dai
import time
import cv2
import json
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from datetime import timedelta

# Default values
RESIZE_RESOLUTION_WIDTH = 640
RESIZE_RESOLUTION_HEIGHT = 480
SPECKLE_FILTER = True
COLOR_CAMERA_RES = dai.ColorCameraProperties.SensorResolution.THE_1080_P
MONO_CAMERA_RES = dai.MonoCameraProperties.SensorResolution.THE_720_P
COLOR_CAMERA_FPS = 30
MONO_CAMERA_FPS = 30
SYNC_THRESHOLD_SECONDS = 0.2

EXTENDED_DISPARITY = False
SUBPIXEL = False
LRCHECK = True
THRESHOLD = 255
BILATERAL_SIGMA = 0

STILL_STREAM_NAME = "still"
CONTROL_STREAM_NAME = "control"
RGB_STREAM_NAME = "rgb"
DEPTH_STREAM_NAME = "depth"
SYNC_STREAM_NAME = "sync"

RGB_TOPIC_NAME = "rgb"
DEPTH_TOPIC_NAME = "depth"
CAMERA_INFO_TOPIC_NAME = "cameraInfo"
ACTION_TOPIC_NAME = "takePhoto"

NODE_NAME = "camera"

ctrl = None
calibData = None
currentTime = None

def flatten_matrix(matrix):
    return [item for sublist in matrix for item in sublist]

def parseConfig():
    try:
        config = open("config.json", "r")
    except:
        print("Cannot load config.json")
        exit(-1)

    configData = json.load(config)

    global RESIZE_RESOLUTION_WIDTH, RESIZE_RESOLUTION_HEIGHT, COLOR_CAMERA_RES
    global MONO_CAMERA_RES, COLOR_CAMERA_FPS, MONO_CAMERA_FPS, SPECKLE_FILTER
    global THRESHOLD, BILATERAL_SIGMA, LRCHECK, EXTENDED_DISPARITY, SUBPIXEL
    global SYNC_THRESHOLD_SECONDS, NODE_NAME, RGB_TOPIC_NAME, DEPTH_TOPIC_NAME
    global CAMERA_INFO_TOPIC_NAME, ACTION_TOPIC_NAME

    if configData["resizeResolution"]["width"]:
        RESIZE_RESOLUTION_WIDTH = configData["resizeResolution"]["width"]
    
    if configData["resizeResolution"]["width"]:
        RESIZE_RESOLUTION_HEIGHT = configData["resizeResolution"]["height"]

    if configData["camera"]["rgb"]["resolution"]:
        if configData["camera"]["rgb"]["resolution"] == "1080":
            COLOR_CAMERA_RES = dai.ColorCameraProperties.SensorResolution.THE_1080_P
        elif configData["camera"]["rgb"]["resolution"] == "4K":
            COLOR_CAMERA_RES = dai.ColorCameraProperties.SensorResolution.THE_4_K
    
    if configData["camera"]["mono"]["resolution"]:
        if configData["camera"]["mono"]["resolution"] == "720":
            MONO_CAMERA_RES = dai.MonoCameraProperties.SensorResolution.THE_720_P
        elif configData["camera"]["mono"]["resolution"] == "400":
            MONO_CAMERA_RES = dai.MonoCameraProperties.SensorResolution.THE_400_P
    
    if configData["camera"]["rgb"]["fpsCap"]:
        COLOR_CAMERA_FPS = configData["camera"]["rgb"]["fpsCap"]
    
    if configData["camera"]["rgb"]["fpsCap"]:
        MONO_CAMERA_FPS = configData["camera"]["mono"]["fpsCap"]

    if configData["stereo"]["speckleFilter"]:
        SPECKLE_FILTER = configData["stereo"]["speckleFilter"]
    
    if configData["stereo"]["confidenceThreshold"]:
        THRESHOLD = configData["stereo"]["confidenceThreshold"]

    if configData["stereo"]["bilateralSigma"]:
        BILATERAL_SIGMA = configData["stereo"]["bilateralSigma"]
    
    if configData["stereo"]["LRCheck"]:
        LRCHECK = configData["stereo"]["LRCheck"]
    
    if configData["stereo"]["extendedDisparity"]:
        EXTENDED_DISPARITY = configData["stereo"]["extendedDisparity"]
    
    if configData["stereo"]["subpixel"]:
        SUBPIXEL = configData["stereo"]["subpixel"]

    if configData["sync"]["syncThresholdSeconds"]:
        SYNC_THRESHOLD_SECONDS = configData["sync"]["syncThresholdSeconds"]
    
    if configData["node"]["name"]:
        NODE_NAME = configData["node"]["name"]
    
    if configData["node"]["rgbTopicName"]:
        RGB_TOPIC_NAME = configData["node"]["rgbTopicName"]
    
    if configData["node"]["depthTopicName"]:
        DEPTH_TOPIC_NAME = configData["node"]["depthTopicName"]
    
    if configData["node"]["cameraInfoTopicName"]:
        CAMERA_INFO_TOPIC_NAME = configData["node"]["cameraInfoTopicName"]

    if configData["node"]["actionTopicName"]:
        ACTION_TOPIC_NAME = configData["node"]["actionTopicName"]


class Camera(Node):

    def __init__(self, qSync, qControl):
        super().__init__(NODE_NAME)
        self.qSync = qSync
        self.qControl = qControl
        self.i = 0
        self.bridge = CvBridge()
        
        # Create the publisher
        self.publisherRGB = self.create_publisher(Image, RGB_TOPIC_NAME, 10)
        self.publisherDepth = self.create_publisher(Image, DEPTH_TOPIC_NAME, 10)
        self.publisherCameraInfo = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC_NAME, 10)
        
        # Create the subscription
        self.create_subscription(String, ACTION_TOPIC_NAME, self.timer_callback, 10)
        print("ROS node configured correctly")
    
    def timer_callback(self, data):
        global currentTime

        print("Sending signal to camera")

        # Create the camera signal to shoot the photo
        self.qControl.send(ctrl)
        
        # Wait until the frames are taken, get the frame and switch it to the correct sending function
        msgGroup = self.qSync.get()

        currentTime = self.get_clock().now().to_msg()
        rgbTs = depthTs = 0
        
        for name, msg in msgGroup:
            if name == STILL_STREAM_NAME:
                print("RGB timestamp:", msg.getTimestamp())
                rgbTs = msg.getTimestamp()
                self.send_frame(msg, self.publisherRGB)

            elif name == DEPTH_STREAM_NAME:
                print("Depth timestamp:", msg.getTimestamp())
                depthTs = msg.getTimestamp()
                self.send_frame(msg, self.publisherDepth)

            else:
                print("[ERROR] Stream doesn't exist")
                exit(-1)
        
        self.send_cameraInfo()
        
        if rgbTs - depthTs >= timedelta(seconds=0) :
            print(f"RGB delay: { abs(rgbTs - depthTs) } \n")
        else:
            print(f"Depth delay: { abs(rgbTs - depthTs) } \n")
        
        self.i += 1

    def send_frame(self, frame, publisher):
        # Getting the frame
        cv_image = frame.getCvFrame()

        # Converting to ROS format and sending it
        try:
            cv_image = cv2.resize(cv_image, (RESIZE_RESOLUTION_WIDTH, RESIZE_RESOLUTION_HEIGHT), interpolation=cv2.INTER_LINEAR)
            image_ROS = self.bridge.cv2_to_imgmsg(cv_image)
            image_ROS.header.stamp = currentTime
            publisher.publish(image_ROS)
        except CvBridgeError as e:
            print(e)
            exit(-1)

    
    def send_cameraInfo(self):
        print("Sending camera info")

        camera_info_msg = CameraInfo()

        camera_info_msg.header.stamp = currentTime
        camera_info_msg.height = RESIZE_RESOLUTION_HEIGHT
        camera_info_msg.width = RESIZE_RESOLUTION_WIDTH
        
        camera_info_msg.k = flatten_matrix(calibData.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A,
            resizeWidth=RESIZE_RESOLUTION_WIDTH,
            resizeHeight=RESIZE_RESOLUTION_HEIGHT))

        self.publisherCameraInfo.publish(camera_info_msg)
        print("Camera info sent")
        


def main(args=None):

    parseConfig()

    print("Configuring pipeline")
    
    pipeline = dai.Pipeline()

    sync = pipeline.create(dai.node.Sync)

    # Setup color camera
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setResolution(COLOR_CAMERA_RES)
    camRgb.setFps(COLOR_CAMERA_FPS)

    # Connect the RGB stream to the sync node
    camRgb.still.link(sync.inputs[STILL_STREAM_NAME])

    # Input for the Color camera (to control the still stream)
    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName(CONTROL_STREAM_NAME)
    xin.out.link(camRgb.inputControl)
    
    
    # Setup monocamera e stereoDepth nodes
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    
    monoRight.setResolution(MONO_CAMERA_RES)
    monoLeft.setResolution(MONO_CAMERA_RES)
    monoRight.setFps(MONO_CAMERA_FPS)
    monoLeft.setFps(MONO_CAMERA_FPS)
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    stereo.initialConfig.PostProcessing.SpeckleFilter.enable = SPECKLE_FILTER
    stereo.initialConfig.setConfidenceThreshold(THRESHOLD)
    stereo.initialConfig.setBilateralFilterSigma(BILATERAL_SIGMA)
    stereo.setLeftRightCheck(LRCHECK)
    stereo.setExtendedDisparity(EXTENDED_DISPARITY)
    stereo.setSubpixel(SUBPIXEL)
    
    # Aligning the output of stereo to the output of color camera
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    # Connect the stereo depth stream to the sync node
    stereo.disparity.link(sync.inputs[DEPTH_STREAM_NAME])

    # 2 frames are considered synced if shoot in a window of 0.5 seconds
    sync.setSyncThreshold(timedelta(seconds = SYNC_THRESHOLD_SECONDS))
    
    xoutSynced = pipeline.create(dai.node.XLinkOut)
    xoutSynced.setStreamName(SYNC_STREAM_NAME)
    sync.out.link(xoutSynced.input)

    global ctrl
    ctrl = dai.CameraControl()
    ctrl.setCaptureStill(True)

    print("Pipeline configuration terminated")

    with dai.Device(pipeline) as device:
        
        # Calibrating the camera
        print("Calibrating the camera")
        
        global calibData
        calibData = device.readCalibration()
        lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.CAM_A)
        if lensPosition:
            camRgb.initialControl.setManualFocus(lensPosition)

        qSync = device.getOutputQueue(name=SYNC_STREAM_NAME, maxSize=1, blocking=False)
        qControl = device.getInputQueue(name=CONTROL_STREAM_NAME)

        time.sleep(0.5)

        # ROS inizialization
        rclpy.init(args=args)

        camera = Camera(qSync, qControl)

        # Starting the callbacks
        try:
            rclpy.spin(camera)
        except KeyboardInterrupt:
            print("Shutting down")

        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
