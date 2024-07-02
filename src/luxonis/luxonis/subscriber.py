import rclpy
import numpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

RGB_TOPIC_NAME = "rgb"
DEPTH_TOPIC_NAME = "depth"
WINDOW_NAME = "RGB/DEPTH"
DEFAULT_DISPARITY = 95

# Percentuale rgb/depth nell'immagine
rgbWeight = 0.4
depthWeight = 0.6

frameRgb = None
frameDisp = None

# Funzione per regolare la percentuale di rgb e depthmap 
def updateBlendWeights(percent_rgb):
    global depthWeight, rgbWeight
    rgbWeight = float(percent_rgb) / 100.0
    depthWeight = 1.0 - rgbWeight

class Receiver(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.bridge = CvBridge()
        self.subscriberRGB_ = self.create_subscription(Image, RGB_TOPIC_NAME, self.callbackRGB, 10)
        self.subscriberDEPTH_ = self.create_subscription(Image, DEPTH_TOPIC_NAME, self.callbackDEPTH, 10)

    def callbackRGB(self, data):
        global frameRgb
        try:
            frameRgb = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            exit(-1)

    def callbackDEPTH(self, data):
        global frameDisp
        try:
            frameDisp = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            exit(-1)

        print("Immagine Depth ricevuta")
        frameDisp = cv2.normalize(frameDisp, None, 0, 255, cv2.NORM_MINMAX)
        frameDisp = numpy.uint8(frameDisp)
        frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)
        print("ColorMap applicata")

        if frameRgb is not None:
            blended = cv2.addWeighted(frameRgb, rgbWeight, frameDisp, depthWeight, 0)
            cv2.imshow(WINDOW_NAME, blended)
            cv2.waitKey(1)

def main(args=None):
    
    rclpy.init(args=args)
    receiver = Receiver()
    
    cv2.namedWindow(WINDOW_NAME)
    cv2.createTrackbar('RGB Depth', WINDOW_NAME, int(rgbWeight * 100), 100, updateBlendWeights)
    

    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        print("Shutting down")

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
