import numpy
import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TOPIC_NAME = "depth"

DEFAULT_DISPARITY = 95


class ReceiverDepth(Node):

    def __init__(self):
        super().__init__('receiverDepth')
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, TOPIC_NAME, self.callback, 10)

    def callback(self, data):
        imageOpenCV = None
        try:
            imageOpenCV = self.bridge.imgmsg_to_cv2(data)
            print("Immagine ricevuta")
            maxDisparity = DEFAULT_DISPARITY
            imageOpenCV = (imageOpenCV * 255. / maxDisparity).astype(numpy.uint8)
            imageOpenCV = cv2.applyColorMap(imageOpenCV, cv2.COLORMAP_JET)
            print("ColorMap applicata")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("finestra",imageOpenCV)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    receiver = ReceiverDepth()

    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        print("Shutting down")

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
