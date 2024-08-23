#!/usr/bin/env python3
import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import SiloMatrix, Coords, XZ, XZs, MotorControl, GripperControl
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import math

torch.cuda.set_device(0)
# Initialize global variables
frame_counter = 0

def callback(colorFrame, depthFrame):
    global frame_counter, start, model, pubSiloMatrix
    frame_counter += 1

    # Process every 5th frame
    if frame_counter % 60 != 0:
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
    start = end

    # Initialize lists to store coordinates of border boxes' center points
    teamBallRealXZs = []
    purpleBallRealXZs = []
    silos = []
    balls = []

    for result in results:
        # Obtain classes name model can detect
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.to("cpu").detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)

            if clsName in ["Silo"]:
                silos.append([x, w])
                continue

            balls.append([x, y, clsName])
            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame)

            if clsName == "purple_ball":
                purpleBallRealXZs.append([real_x, depth])
                continue

            teamBallRealXZs.append([real_x, depth])

    if not teamBallRealXZs:
        return

    def sortByFirstEle(e):
        return e[0]
        
    silos.sort(key=sortByFirstEle)
    silosBound = [[silo[0] - silo[1] // 2, silo[0] + silo[1] // 2] for silo in silos]

    siloMatrix_y = [[[0, "empty"], [0, "empty"], [0, "empty"]] for _ in range(5)]
    siloNum = 0
    
    for siloBound in silosBound:
        lower, upper = siloBound
        layer = 0
        for ball in balls:
            if layer == 3:
                break
            x, y, clsName = ball
            if lower <= x <= upper:
                siloMatrix_y[siloNum][layer] = [y, clsName]
                layer += 1
        siloNum += 1

    for silo in siloMatrix_y:
        silo.sort(key=sortByFirstEle)
        for ball in silo:
            if ball[0] == 0:
                silo.remove(ball)
                silo.append(ball)

    siloNum = 0
    for silo in siloMatrix_y:
        layer = 0
        for ball in silo:
            if ball[1] == "red_ball":
                color = 1
            elif ball[1] == "blue_ball":
                color = 2
            elif ball[1] == "purple_ball":
                color = 3
            else:
                color = 0
            siloMatrix_y[siloNum][layer] = color
            layer += 1
        siloNum += 1

    siloMatrixMsg = SiloMatrix()
    siloMatrixMsg.silo1 = siloMatrix_y[0]
    siloMatrixMsg.silo2 = siloMatrix_y[1]
    siloMatrixMsg.silo3 = siloMatrix_y[2]
    siloMatrixMsg.silo4 = siloMatrix_y[3]
    siloMatrixMsg.silo5 = siloMatrix_y[4]
    
    print(siloMatrixMsg)
    pubSiloMatrix.publish(siloMatrixMsg)

def getDepth(x, y, conf, clsName, depthFrame):
    window_size = 5
    pad_size = (window_size - 1) // 2
    padded_img = cv2.copyMakeBorder(depthFrame, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=0)
    pix_in_win = padded_img[y - pad_size : y + pad_size + 1, x - pad_size : x + pad_size + 1]
    depth = pix_in_win.max()
    print(f"Depth value at ({x}, {y}) is {depth}, cls: {clsName}, conf: {conf}")
    return int(depth)

def calcX(depth, x, colorFrame):
    resolution_w = colorFrame.shape[1]
    center_x = resolution_w // 2
    focal = 580
    real_x = (x - center_x) * (depth / focal)
    return int(real_x)

if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/best.pt")

    gripperState = "o"
    gripperArmState = "forward"
    pubSiloMatrix = rospy.Publisher("silo_matrix", SiloMatrix, queue_size=10)
    pubGripperControl = rospy.Publisher("gripper_control", GripperControl, queue_size=10)
    pubMotorControl = rospy.Publisher("motor_control", MotorControl, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([colorSub, depthSub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

