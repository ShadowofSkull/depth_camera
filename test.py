#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO


def getDepth(data):
    print("test")
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
        print(e)


def inner(data):
    print("rstest")
    for coord in data:
        # Get x y from convertImg prob use publisher and read from topic
        x, y = coord
        print(x, y)
        print("why")
        # print(frame[y][x])
    # cv2.imshow("frame", frame)
    # cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("depth")
    xYSub = rospy.Subscriber("/chatter", Int16MultiArray, callback=inner)

    rospy.spin()
