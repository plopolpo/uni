import cv2
import depthai as dai
import numpy
import collections
import time

pipeline = dai.Pipeline()

# Setup color camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("isp")

camRgb.isp.link(xout.input)

with dai.Device(pipeline) as OAKD:

    cv2.namedWindow("prova")

    print('Usb speed: ', OAKD.getUsbSpeed().name)
    print('Device name: ', OAKD.getDeviceName())
    print('Device information: ', OAKD.getDeviceInfo())

    # Prende i frame dalla coda di output del Link-out
    queueRGB = OAKD.getOutputQueue(name="isp", 
                            maxSize=1, 
                            blocking=False)

    while True:   
        frameRgb = queueRGB.get().getCvFrame()

        cv2.imshow("prova", frameRgb)

        time.sleep(0.5)

        # Premere 'q' per uscire
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            exit()
