from datetime import timedelta
import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import depthai as dai
import time

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

calibData = None

class Camera(Node):

    def __init__(self, qSync, qControl):
        super().__init__('camera')
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
        print("Sending signal to camera")

        # Create the camera signal to shoot the photo
        ctrl = dai.CameraControl()
        ctrl.setCaptureStill(True)
        self.qControl.send(ctrl)

        # Wait until the frames are taken
        while not self.qSync.has():
            continue
        
        # Get the frame and switch it to the correct sending function
        msgGroup = self.qSync.get()
        for name, msg in msgGroup:
            if name == STILL_STREAM_NAME:
                self.send_rgb(msg)

            elif name == DEPTH_STREAM_NAME:
                self.send_depth(msg)

            else:
                print("[ERROR] Stream doesn't exist")
                exit(-1)
        
        self.send_cameraInfo()

        self.i += 1
        

    def send_rgb(self, frame):
        print("Sending RGB")

        # Getting the frame
        cv_image = frame.getCvFrame()

        print(f"OpenCV image extracted with dimension: {cv_image.shape}")

        # Converting to ROS format and sending it
        try:
            image_ROS = self.bridge.cv2_to_imgmsg(cv_image)
            print("OpenCV image converted to ROS format")
            self.publisherRGB.publish(image_ROS)
            print(f"RGB n {self.i} sent")
        except CvBridgeError as e:
            print(e)

    
    def send_depth(self, frame):
        print("Sending Depth")

        # Getting the frame
        cv_image = frame.getCvFrame()

        print(f"OpenCV image extracted with dimension: {cv_image.shape}")

        # Converting to ROS format and sending it
        try:
            image_ROS = self.bridge.cv2_to_imgmsg(cv_image)
            print("OpenCV image converted to ROS format")
            self.publisherDepth.publish(image_ROS)
            print(f"Depth n {self.i} sent")
        except CvBridgeError as e:
            print(e)

    
    def send_cameraInfo(self):
        print("Sending camera info")



def main(args=None):

    print("Configuring pipeline")
    
    pipeline = dai.Pipeline()

    sync = pipeline.create(dai.node.Sync)

    # Setup color camera
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

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
    
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    stereo.initialConfig.PostProcessing.SpeckleFilter.enable = True
    stereo.initialConfig.setConfidenceThreshold(THRESHOLD)
    stereo.initialConfig.setBilateralFilterSigma(BILATERAL_SIGMA)
    stereo.setLeftRightCheck(LRCHECK)
    stereo.setExtendedDisparity(EXTENDED_DISPARITY)
    stereo.setSubpixel(SUBPIXEL)

    # Aligning the output of stereo to the output of color camera
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    # Connect the stereo depth stream to the sync node
    stereo.depth.link(sync.inputs[DEPTH_STREAM_NAME])

    # 2 frames are considered synced if shoot in a window of 0.5 seconds
    sync.setSyncThreshold(timedelta(seconds = 0.5))
    
    xoutSynced = pipeline.create(dai.node.XLinkOut)
    xoutSynced.setStreamName(SYNC_STREAM_NAME)
    sync.out.link(xoutSynced.input)

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
