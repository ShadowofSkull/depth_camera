#!/usr/bin/env python3
import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords, XZ, XZs, MotorControl, GripperControl
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

    # Process every 5th frame
    if frame_counter % 60 != 0:
    # if frame_counter % 5 != 0:
        return

    # Convert ROS msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
    except CvBridgeError as e:
        print(e)
        return

    # Run YOLOv8 inference on the colorFrame
    results = model(colorFrame)

    # Check interval between callback
    end = time.time()
    print(f"Time between frames: {end - start:.2f} seconds")
    start = end

    # Initialise list to store coordinates of boxes centre point
    teamBallRealXZs = []
    purpleBallRealXZs = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        print(f"name: {names}")
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.to("cpu").detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # Skip if clsName is not balls
            if clsName not in ["red_ball", "blue_ball", "purple_ball"] or conf < 50:
               continue
 
            # Obtain xy which is centre coords of
            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)

            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame)

            if clsName == "purple_ball":
                purpleBallRealXZs.append([real_x, depth])
                print(f"purple realx:{real_x}, depth:{depth}")
                continue

            teamBallRealXZs.append([real_x, depth])
            print(f"realx:{real_x}, depth:{depth}")

    if (not teamBallRealXZs):
        print("No team ball found")
        return

    
    while True:
        teamBall = findClosestBall(teamBallRealXZs)
        tBallX, tBallZ = teamBall[0], teamBall[1]
        purpleBall = findClosestBall(purpleBallRealXZs)
        pBallX, pBallZ = purpleBall[0], purpleBall[1]
        if tBallX < pBallX - 300 or tBallX > pBallX + 300:
            print("balls not on same x axis")
            break
        
        if tBallZ <= pBallZ:
            print("team ball in front of purple ball")
            break

        print("purple ball blocking")
        teamBallRealXZs.remove(teamBall)
        
        if not teamBallRealXZs:
            print("No suitable team ball found")
            return
    
    publishControl(teamBall)


def findClosestBall(ballRealXZs):        
    
    closestBallXZ = []
    for i in range(len(ballRealXZs)):
        if closestBallXZ == []:
            closestBallXZ.append(ballRealXZs[i][0])
            closestBallXZ.append(ballRealXZs[i][1])
        elif ballRealXZs[i][1] < closestBallXZ[1]:
            closestBallXZ[0] = ballRealXZs[i][0]
            closestBallXZ[1] = ballRealXZs[i][1]
            
    if not closestBallXZ:
        print("list empty")
        return []

def publishControl(closestBallXZ):
    # Motor publish
    motorMsg = MotorControl()
    print(closestBallXZ[0])
    motorMsg.horizontal = closestBallXZ[0]
    motorMsg.forward = closestBallXZ[1]
    print(motorMsg)
    pubMotorControl.publish(motorMsg)
    # Gripper publish
    global gripperState, gripperArmState

    gripperMsg = GripperControl()
    if gripperState == "o":
        gripperState = "c"
        gripperMsg.grip = gripperState
    elif gripperState == "c":
        gripperState = "o"
        gripperMsg.grip = gripperState

    if gripperArmState == "forward":
        gripperArmState = "backward"
        gripperMsg.flip = gripperArmState
    elif gripperArmState == "backward":
        gripperArmState = "forward"
        gripperMsg.flip = gripperArmState

    print(gripperMsg)
    pubGripperControl.publish(gripperMsg)




def calcX(depth, x, colorFrame):
    resolution_w = colorFrame.shape[1]
    center_x = resolution_w // 2
    # need to obtain using f = (xpixel - center x) * distance / real x 462 -320 * 486 / 1400
    focal = 580
    real_x = (x - center_x) * (depth / focal)
    return int(real_x)



def getDepth(x, y, conf, clsName, depthFrame):
    # Create a max spatial filter to get the max value in a 3x3 or bigger window
    window_size = 5
    # window_size change pattern = 3 + 2x, x = 0, 1, 2,...
    # pad_size change pattern = 1 + x, x = 0, 1, 2,...
    # Do some substitution to get the formula
    # pad = (win - 1) // 2
    pad_size = (window_size - 1) // 2
    padded_img = cv2.copyMakeBorder(
        depthFrame, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=0
    )
    pix_in_win = padded_img[
        y - pad_size : y + pad_size + 1, x - pad_size : x + pad_size + 1
    ]
    depth = pix_in_win.max()
    # if they store nan value in pixel need use this
    # depth = np.nanmax(pix_in_win)

    print(f"Depth value at ({x}, {y}) is {depth}, cls: {clsName}, conf: {conf}")
    return int(depth)


if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/best.pt")

    gripperState = "o"
    gripperArmState = "forward"
    print("done init")

    pubGripperControl = rospy.Publisher("gripper_control", GripperControl, queue_size=10)
    pubMotorControl = rospy.Publisher("motor_control", MotorControl, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer(
        [colorSub, depthSub], 10, 0.1, allow_headerless=True
    )
    # ts = message_filters.TimeSynchronizer([colorSub, depthSub], 10)

    ts.registerCallback(callback)
    print("done sub/pub, callback")
    rospy.spin()
