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

    global frame_counter, model, gripperArmState
    frame_counter += 1

    # Process every 60th frame
    if frame_counter % 60 != 0:
        return

     # Convert ROS msg to cv nparray that's suitable for the model
    bridge = CvBridge()
    try:
        colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
    except CvBridgeError as e:
        print(e)
        return

    # Run YOLOv8 inference on the colorFrame
    results = model(colorFrame)

    # Initialize lists for storing detection results
    teamBallRealXZs = []
    purpleBallRealXZs = []
    detectionName = ""
    balls = []

    for result in results:
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            xywh = xywh.to("cpu").detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            if conf <= 50:
                continue

            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)

            if clsName in ["couch"]:
                detectionName = clsName
                continue

            if clsName in ["ball", "red_ball", "purple_ball"]:
                balls.append([x, y, clsName])

            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame)

            if clsName == "purple_ball":
                purpleBallRealXZs.append([real_x, depth])
                continue

            teamBallRealXZs.append([real_x, depth])

    # Publish to gripper and motor depending on whether we are facing balls or couch
    if detectionName == "couch":
        couchPublishControl()
    else:
        if not teamBallRealXZs and not purpleBallRealXZs:
            print("No balls found, robot stop")
            stopRobot()
            return

        closestTeamBallXZ = findClosestBall(teamBallRealXZs)
        closestPurpleBallXZ = findClosestBall(purpleBallRealXZs)

        if closestTeamBallXZ is None and closestPurpleBallXZ is None:
            print("No closest balls found, robot stop")
            stopRobot()
            return

        ballPublishControl(closestTeamBallXZ, closestPurpleBallXZ)


def findClosestBall(ballRealXZs):
    if not ballRealXZs:
        return None

    closestBallXZ = ballRealXZs[0]
    for ball in ballRealXZs:
        if ball[1] < closestBallXZ[1]:
            closestBallXZ = ball

    return closestBallXZ


def couchPublishControl():
    print("Publishing for couch")
    # Motor publish
    motorMsg = MotorControl()
    motorMsg.x = 100  # Example value, adjust as needed
    motorMsg.z = 50  # Example value, adjust as needed
    pubMotorControl.publish(motorMsg)

    # Gripper publish
    gripperMsg = GripperControl()
    gripperMsg.grip = "o"  # Open gripper
    gripperMsg.flip = "forward"  # Flip arm forward
    pubGripperControl.publish(gripperMsg)

    # Delay to simulate action
    time.sleep(5)


def ballPublishControl(closestTeamBallXZ, closestPurpleBallXZ):
    print("Publishing for balls")
    # Motor publish
    motorMsg = MotorControl()
    if closestTeamBallXZ:
        motorMsg.x = closestTeamBallXZ[0]
        motorMsg.z = closestTeamBallXZ[1]
    elif closestPurpleBallXZ:
        motorMsg.x = closestPurpleBallXZ[0]
        motorMsg.z = closestPurpleBallXZ[1]
    pubMotorControl.publish(motorMsg)

    # Gripper publish
    gripperMsg = GripperControl()
    gripperMsg.grip = "o"  # Open gripper
    gripperMsg.flip = "forward"  # Flip arm forward
    pubGripperControl.publish(gripperMsg)

    # Delay to simulate action
    time.sleep(5)


def stopRobot():
    motorMsg = MotorControl()
    motorMsg.x = 0
    motorMsg.z = 0
    pubMotorControl.publish(motorMsg)


def calcX(depth, x, colorFrame):
    resolution_w = colorFrame.shape[1]
    center_x = resolution_w // 2
    focal = 580
    real_x = (x - center_x) * (depth / focal)
    return int(real_x)


def getDepth(x, y, conf, clsName, depthFrame):
    window_size = 5
    pad_size = (window_size - 1) // 2
    padded_img = cv2.copyMakeBorder(
        depthFrame, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=0
    )
    pix_in_win = padded_img[
        y - pad_size : y + pad_size + 1, x - pad_size : x + pad_size + 1
    ]
    depth = pix_in_win.max()

    print(f"Depth value at ({x}, {y}) is {depth}, cls: {clsName}, conf: {conf}")
    return int(depth)


if __name__ == "__main__":
    rospy.init_node("detect")
    model = YOLO("./best.pt")
    frame_counter = 0

    pubGripperControl = rospy.Publisher(
        "gripper_control", GripperControl, queue_size=10
    )
    pubMotorControl = rospy.Publisher("motor_control", MotorControl, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer(
        [colorSub, depthSub], 10, 0.1, allow_headerless=True
    )
    ts.registerCallback(callback)

    rospy.spin()
