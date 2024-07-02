import rclpy
import numpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

RGB_TOPIC_NAME = "rgb"
DEPTH_TOPIC_NAME = "depth"
WINDOW_NAME = "rgb depth"
WINDOW_SIZE = (848, 480)
DEFAULT_DISPARITY = 95

# Percentuale rgb/depth nell'immagine
rgbWeight = 0.4
depthWeight = 0.6

frameRgb = None
frameDisp = None

class Receiver(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.bridge = CvBridge()
        self.subscriberRGB_ = self.create_subscription(Image, RGB_TOPIC_NAME, self.callbackRGB, 10)

    def callbackRGB(self, data):
        global frameRgb
        try:
            frameRgb = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            exit(-1)

        print(f"OpenCV image extracted with dimension: {frameRgb.shape}")
        print("Immagine RGB ricevuta")
        cv2.imshow(WINDOW_NAME, frameRgb)
        cv2.waitKey(1)

def main(args=None):

    
    rclpy.init(args=args)
    receiver = Receiver()
    
    # Crea la trackbar per regolare depth - rgb
    cv2.namedWindow(WINDOW_NAME)

    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        print("Shutting down")

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
