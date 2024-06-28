import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TOPIC_NAME = "rgb"
TIMER_PERIOD = 0.5

class ReceiverRGB(Node):

    def __init__(self):
        super().__init__('fotocamera')
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, TOPIC_NAME, self.callback, 10)
        self.i = 0

    def callback(self, data):
        try:
            imageOpenCV = self.bridge.imgmsg_to_cv2(data)
            print("Immagine ricevuta")
            print(type(imageOpenCV))
            cv2.imshow("finestra", imageOpenCV)
            self.i += 1
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
