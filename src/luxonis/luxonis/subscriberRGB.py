import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TOPIC_NAME = "rgb"

class ReceiverRGB(Node):

    def __init__(self):
        super().__init__('subscriberRGB')
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, TOPIC_NAME, self.callback, 10)

    def callback(self, data):
        try:
            imageOpenCV = self.bridge.imgmsg_to_cv2(data)
            print("Immagine ricevuta")
            print(type(imageOpenCV))
            print(f"OpenCV image extracted with dimension: {imageOpenCV.shape}")
            cv2.imshow("finestra", imageOpenCV)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    
    receiver = ReceiverRGB()

    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        print("Shutting down")

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
