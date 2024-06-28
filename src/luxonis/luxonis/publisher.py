import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import depthai as dai
import time

RGB_TOPIC_NAME = "rgb"
DEPTH_TOPIC_NAME = "depth"
ACTION_TOPIC_NAME = "takePhoto"
TIMER_PERIOD = 2

latestFrame = None

class Camera(Node):

    def __init__(self, qStill, qControl):
        super().__init__('fotocamera')
        self.qStill = qStill
        self.qControl = qControl

        self.bridge = CvBridge()
        self.publisherRGB = self.create_publisher(Image, RGB_TOPIC_NAME, 10)
        self.publisherDepth = self.create_publisher(Image, DEPTH_TOPIC_NAME, 10)
        self.subscriber_ = self.create_subscription(String, ACTION_TOPIC_NAME, self.timer_callback, 10)
        print("Inizializzazione nodo ros terminata")
    

    def send_rgb(self):
        print("Sending RGB")
        ctrl = dai.CameraControl()
        ctrl.setCaptureStill(True)
        self.qControl.send(ctrl)

        print("Attesa")
        while not self.qStill.has():
            continue
        
        cv_image = self.qStill.get().getCvFrame()

        print(f"Immagine openCV estratta con dimensioni: {cv_image.shape}")

        try:
            image_ROS = self.bridge.cv2_to_imgmsg(cv_image)
            print("Immagine OpenCV convertita in formato ROS")
            self.publisherRGB.publish(image_ROS)
            print("Immagine inviata")
        except CvBridgeError as e:
            print(e)

    
    def send_depth(self):
        print("Sending Depth")

    def timer_callback(self, data):
        self.send_rgb()
        self.send_depth()

    

def main(args=None):

    print("Configurazione pipeline")
    
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("control")
    xin.out.link(camRgb.inputControl)

    xoutStill = pipeline.create(dai.node.XLinkOut)
    xoutStill.setStreamName("still")
    camRgb.still.link(xoutStill.input)

    print("Configurazione pipeline terminata")

    with dai.Device(pipeline) as device:

        qStill = device.getOutputQueue(name="still")
        qControl = device.getInputQueue(name="control")

        time.sleep(0.5)

        # ROS part
        rclpy.init(args=args)

        camera = Camera(qStill, qControl)

        try:
            rclpy.spin(camera)
        except KeyboardInterrupt:
            print("Shutting down")

        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
