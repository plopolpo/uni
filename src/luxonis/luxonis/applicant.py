import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_NAME = "takePhoto"
TIMER_PERIOD = 2

class CameraRGB(Node):

    def __init__(self):
        super().__init__('applicant')

        self.publisher_ = self.create_publisher(String, TOPIC_NAME, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        

    def timer_callback(self):
        msg = String()
        self.publisher_.publish(msg)

def main(args=None):

    # ROS part
    rclpy.init(args=args)

    camera = CameraRGB()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        print("Shutting down")

    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
