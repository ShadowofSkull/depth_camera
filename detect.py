#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO


def convertImg(data):
    # Convert ros msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Run YOLOv8 inference on the frame
    results = model(frame)
    end = time.time()
    print(end - start)
    # Initialise list to store coordinates of boxes centre point
    depthCoords = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.numpy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # Obtain xy which is centre coords of
            x, y, w, h = xywh[0]
            # x y is the centre pixel of the box
            x = int(x)
            y = int(y)
            coord = Coords()
            coord.x = x
            coord.y = y
            coord.conf = conf
            coord.cls = clsName
            # print(x, y, conf, clsName)
            depthCoords.append(coord)
            # Visualize the results on the frame
            annotated_frame = result.plot()
            # Draw a red dot at the centre of the box illustration purpose
            annotated_frame[y : y + 5, x : x + 5] = [0, 0, 255]
        # Display the annotated frame
        if boxes == None:
            continue
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(1)
    # Publish xy to a topic so depth node can use it
    msg.coords = depthCoords
    # print(msg.coords)
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("detect")
    msg = CoordsMatrix()
    model = YOLO("yolov8m.pt")
    # Detect every 6 frames
    rate = rospy.Rate(0.2)
    print("done init")
    pub = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg)

    start = time.time()
    # rospy.spin()
    while not rospy.is_shutdown():
        rate.sleep()
