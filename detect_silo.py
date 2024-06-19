#!/usr/bin/env python3
import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
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
    # print(f"Time between frames: {end - start:.2f} seconds")
    start = end

    # Initialise list to store coordinates of boxes centre point
    teamBallRealXZs = []
    purpleBallRealXZs = []
    silos = []
    silosRealXZ = []
    balls = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        # print(f"name: {names}")
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.to("cpu").detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # if conf < 50:
            # continue
            # Skip if clsName is not balls

            # Obtain xy which is centre coords of
            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)

            if clsName in ["Silo"]:
                # lower upper x bound of 5 silos
                print(f"x: {x}, w:{w}")
                silos.append([x, w, y])
                depth = getDepth(x, y, conf, clsName, depthFrame)
                real_x = calcX(depth, x, colorFrame)
                silosRealXZ.append([real_x, depth])
                continue

            # For processing balls in silos later
            balls.append([x, y, clsName])

            # To get real x and depth for balls on the floor
            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame)
            # Separate ball by purple and team color
            if clsName == "purple_ball":
                purpleBallRealXZs.append([real_x, depth])
                # print(f"purple realx:{real_x}, depth:{depth}")
                continue

            teamBallRealXZs.append([real_x, depth])
            # print(f"realx:{real_x}, depth:{depth}")

    if not teamBallRealXZs:
        print("No team ball found, robot stop")
        return

    siloMatrix = createSiloMatrix(silos, silosRealXZ, balls)
    # Determine best silo to place the ball based on priorities
    # findBestSilo(siloMatrix, team_color)

    # def check_v_goal(matrix):
    #     team_color = 1  # Assuming 1 represents team color
    #     v_goal_count = 0

    #     for silo in matrix:
    #         if silo[2] == team_color and silo.count(team_color) >= 2:
    #             v_goal_count += 1

    #     if v_goal_count >= 3:
    #         rospy.loginfo("V Goal achieved! Team wins!")
    #         # Additional actions for V Goal can be added here

    # def findBestSilo(siloMatrix, team_color=1):
    #     best_silo = -1
    #     priority_1 = []
    #     priority_2 = []
    #     priority_3 = []

    #     for i, silo in enumerate(siloMatrix):
    #         if silo[1] == 2:  # Priority 1
    #             priority_1.append(i)
    #         elif silo[1] == 1:  # Priority 2
    #             priority_2.append(i)
    #         elif silo[0] == 0:  # Priority 3
    #             priority_3.append(i)

    #     if priority_1:
    #         best_silo = priority_1[0]  # Choose first silo with opponentBall in second row
    #     elif priority_2:
    #         best_silo = priority_2[0]  # Choose first silo with teamBall in second row
    #     elif priority_3:
    #         best_silo = priority_3[0]  # Choose first empty silo

    #     # Check for V Goal condition
    #     check_v_goal(siloMatrix)
    #     return best_silo

    if not silos:
        publishControl(teamBallRealXZs[0], siloMatrix)
    else:
        publishControl()


def createSiloMatrix(silos, silosRealXZ, balls):
    # make it so that it sort only using x value
    def sortByFirstEle(e):
        return e[0]

    # it is 2d array so two values and by providing key to only sort using the first val which is x
    silos.sort(key=sortByFirstEle)
    silosRealXZ.sort(key=sortByFirstEle)

    # Assigning lower and upper x axis bound of each silo
    silosBound = [[silo[0] - silo[1] // 2, silo[0] + silo[1] // 2] for silo in silos]
    print(silosBound)
    # Setting the matrix as empty, 9999 is so when using sorting algo it goes to the back as it represent the top of silo
    siloMatrix = [[[9999, "empty"], [9999, "empty"], [9999, "empty"]] for _ in range(5)]

    # Checking which silo the ball belong to by checking if its center x axis is within silo boundary
    siloNum = 0
    for siloBound in silosBound:
        lower, upper = siloBound
        layer = 0
        for ball in balls:
            if layer == 3:
                break
            x, y, clsName = ball
            if x >= lower and x <= upper:
                print(siloNum)
                siloMatrix[siloNum][layer] = [y, clsName]
                layer += 1
        siloNum += 1

    print(f"{siloMatrix}\n")
    # Sort the y value in each silos in ascending order
    for silo in siloMatrix:
        silo.sort()

    print(f"{siloMatrix}\n")

    # Replacing the matrix with single int values that represent different ball color
    siloNum = 0
    for silo in siloMatrix:
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
            siloMatrix[siloNum][layer] = color
            layer += 1
        siloNum += 1
    return siloMatrix


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


def publishControl(coordinatesToApproach, siloMatrix):
    # Motor publish
    motorMsg = MotorControl()
    print(coordinatesToApproach[0])
    motorMsg.horizontal = coordinatesToApproach[0]
    motorMsg.forward = coordinatesToApproach[1]
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

    siloMatrixMsg = SiloMatrix()
    siloMatrixMsg.silo1 = siloMatrix[0]
    siloMatrixMsg.silo2 = siloMatrix[1]
    siloMatrixMsg.silo3 = siloMatrix[2]
    siloMatrixMsg.silo4 = siloMatrix[3]
    siloMatrixMsg.silo5 = siloMatrix[4]

    print(siloMatrixMsg)
    pubSiloMatrix.publish(siloMatrixMsg)
    # To control change of decision
    time.sleep(1)


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
    pubSiloMatrix = rospy.Publisher("silo_matrix", SiloMatrix, queue_size=10)
    pubGripperControl = rospy.Publisher(
        "gripper_control", GripperControl, queue_size=10
    )
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
