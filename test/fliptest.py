#!/usr/bin/env python3
import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from astra_camera.msg import MotorControl, GripperControl, SiloMatrix
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import math

torch.cuda.set_device(0)

# Initialize global variables
frame_counter = 0


def callback(colorFrame, depthFrame):
    global frame_counter, start, model, pubCoords
    frame_counter += 1



    # Convert ROS msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        if gripperArmState == "backward":
            colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
            depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
            colorFrame = cv2.flip(colorFrame, -1)
            depthFrame = cv2.flip(depthFrame, -1)
        elif gripperArmState == "forward":
            colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
            depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
    except CvBridgeError as e:
        print(e)
        return

    try:
        cv2.imshow("tses", colorFrame)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
    except:
        print("failed")


if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/best.pt")

    # y for detected, n for no detection
    gripperArmState = "backward"
   
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    # irSub = rospy.Subscriber("BallInGripper", String, irCallback)
    # ultrasonicSub = rospy.Subscriber("InFrontOfSilo", String, ultrasonicCallback)
    ts = message_filters.ApproximateTimeSynchronizer(
        [colorSub, depthSub], 10, 0.1, allow_headerless=True
    )
    # ts = message_filters.TimeSynchronizer([colorSub, depthSub], 10)

    ts.registerCallback(callback)
    print("done sub/pub, callback")
    rospy.spin()
