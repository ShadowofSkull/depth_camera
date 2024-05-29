#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

# from std_msgs.msg import Int16MultiArray
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
    # Initialise list to store coordinates of boxes centre point
    depthCoords = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.numpy()
            # print(
            # f"xy coord and width height: {xywh}\nclass of box: {box.cls}\nconfidence: {box.conf[0]}\n\n"
            # )
            # Obtain xy which is centre coords of
            # print(f"xywh numpy: {xywh[0]}")
            x, y, w, h = xywh[0]
            # x y is the centre pixel of the box
            x = int(x)
            y = int(y)
            dict = Coords()
            dict.x = x
            dict.y = y
            print(x, y)
            # depthCoords.append([x, y])
            # if not (x == None or y == None or x == 0 or y == 0):
            depthCoords.append(dict)

            # Visualize the results on the frame
            # annotated_frame = result.plot()
            # Draw a red dot at the centre of the box illustration purpose
            # annotated_frame[y : y + 5, x : x + 5] = [0, 0, 255]
        # Display the annotated frame
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(10)
    # Publish xy to a topic so depth node can use it
    msg.coords = depthCoords
    print(msg)
    print(msg.coords)
    pub.publish(msg)
    # rate.sleep()


if __name__ == "__main__":
    rospy.init_node("detect")
    msg = CoordsMatrix()
    model = YOLO("yolov8m.pt")
    rate = rospy.Rate(30)
    print("done init")
    pub = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg)

    rospy.spin()
