import depthai
import numpy
import cv2
import time
from datetime import timedelta

FPS = 40
COLOR_CAMERA_QUEUE_SIZE = 10
TEXT_OFFSET = 20
TEXT_COLOR = (255, 255, 255)
QUEUE_BLOCKING = False
CAMERA_INTERLEAVED = False
STREAM_NAME = "output"
RGB_STREAM_NAME = "rgb"
DISPARITY_STREAM_NAME = "disparity"
WINDOW_NAME = "rgb/depth"
MS_THRESHOLD = 50

EXTENDED_DISPARITY = False
SUBPIXEL = False
LRCHECK = True
THRESHOLD = 255
BILATERAL_SIGMA = 0

# Percentuale rgb/depth nell'immagine
rgbWeight = 0.4
depthWeight = 0.6

# Funzione per regolare la percentuale di rgb e depthmap 
def updateBlendWeights(percent_rgb):
    global depthWeight, rgbWeight
    rgbWeight = float(percent_rgb) / 100.0
    depthWeight = 1.0 - rgbWeight

class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-100:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])

pipeline = depthai.Pipeline()

monoLeft = pipeline.create(depthai.node.MonoCamera)
monoRight = pipeline.create(depthai.node.MonoCamera)
RGBCamera = pipeline.create(depthai.node.ColorCamera)
stereo = pipeline.create(depthai.node.StereoDepth)
sync = pipeline.create(depthai.node.Sync)

monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(depthai.CameraBoardSocket.CAM_C)
monoLeft.setBoardSocket(depthai.CameraBoardSocket.CAM_B)
monoRight.setFps(FPS)
monoLeft.setFps(FPS)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

stereo.initialConfig.setMedianFilter(depthai.MedianFilter.MEDIAN_OFF)
stereo.initialConfig.PostProcessing.SpeckleFilter.enable = True
stereo.initialConfig.setConfidenceThreshold(THRESHOLD)
stereo.initialConfig.setBilateralFilterSigma(BILATERAL_SIGMA)
stereo.setLeftRightCheck(LRCHECK)
stereo.setExtendedDisparity(EXTENDED_DISPARITY)
stereo.setSubpixel(SUBPIXEL)
stereo.setDepthAlign(depthai.CameraBoardSocket.CAM_A)

stereo.disparity.link(sync.inputs[DISPARITY_STREAM_NAME])

RGBCamera.setBoardSocket(depthai.CameraBoardSocket.CAM_A)
RGBCamera.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
RGBCamera.setInterleaved(CAMERA_INTERLEAVED)
RGBCamera.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
RGBCamera.setFps(FPS)
RGBCamera.isp.link(sync.inputs[RGB_STREAM_NAME])
RGBCamera.setIspScale(2, 3)

sync.setSyncThreshold(timedelta(seconds=(1 / FPS) * 0.5))

xoutGrp = pipeline.create(depthai.node.XLinkOut)
xoutGrp.setStreamName(STREAM_NAME)
sync.out.link(xoutGrp.input)


with depthai.Device(pipeline) as OAKD:
    
    print('Connection type: ', OAKD.getUsbSpeed().name)
    print('Device name: ', OAKD.getDeviceName())
    print('Device information: ', OAKD.getDeviceInfo())

    frameRgb = None
    frameDepth = None

    cv2.namedWindow(WINDOW_NAME)
    cv2.createTrackbar('R%D', WINDOW_NAME, int(rgbWeight * 100), 100, updateBlendWeights)

    queue = OAKD.getOutputQueue(
        STREAM_NAME, 
        COLOR_CAMERA_QUEUE_SIZE, 
        QUEUE_BLOCKING)

    maxDisparity = 95

    calibData = OAKD.readCalibration2()
    lensPosition = calibData.getLensPosition(depthai.CameraBoardSocket.CAM_A)
    if lensPosition:
        RGBCamera.initialControl.setManualFocus(lensPosition)

    fpsCounter = FPSCounter()
    while True:
        
        msgGrp = queue.tryGet()

        if msgGrp is not None:
            fpsCounter.tick()

            for name, msg in msgGrp:

                if name == DISPARITY_STREAM_NAME:
                    frameDisp = (msg.getFrame() * 255. / maxDisparity).astype(numpy.uint8)
                    frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)

                elif name == RGB_STREAM_NAME:
                    frameRgb = msg.getCvFrame()

                else:
                    print("[ERROR] Stream doesn't exist")
                    exit(-1)

            if len(frameDisp.shape) < 3:
                frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(frameRgb, rgbWeight, frameDisp, depthWeight, 0)

            cv2.putText(blended, f"FPS: {int(fpsCounter.getFps()*1000)/1000}", 
                            (0, 2*TEXT_OFFSET), cv2.FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOR, 2, cv2.LINE_AA)
            
            cv2.imshow(WINDOW_NAME, blended)
            
            frameRgb = None
            frameDisp = None
            frameDepth = None

        else:
            if cv2.waitKey(1) == ord("q"):
                cv2.destroyAllWindows()
                exit(0)
                break
