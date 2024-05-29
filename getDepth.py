#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO


def getCoords(data):
    global coords
    coords = data.data
    for coord in coords:
        print(coord)


def getDepth(data):
    global coords
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
        print(e)

    for coord in coords:
        # Get x y from convertImg prob use publisher and read from topic
        x, y = coord
        print(frame[y][x])

    # cv2.imshow("frame", frame)
    # cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("depth")
    # Temp initialise it as empty arr for later sharing of coords
    coords = []
    coordsSub = rospy.Subscriber("/coords", Int16MultiArray, callback=getCoords)
    depthSub = rospy.Subscriber("/camera/depth/image_raw", Image, callback=getDepth)
    rospy.spin()
