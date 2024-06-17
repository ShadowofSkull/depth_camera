import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords, XZ, XZs, MotorControl, GripperControl
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import math

torch.cuda.set_device(0)

# Initialize global variables
frame_counter = 0
silo_matrix = np.zeros((3, 5), dtype=int)  # Initialize the 3x5 matrix
silo_boxes = []  # List to store silo bounding boxes [(x_min, y_min, x_max, y_max)]

def callback(colorFrame, depthFrame):
    global frame_counter, start, model, pubCoords, pubSiloState, silo_boxes
    frame_counter += 1

    # Process every 5th frame
    if frame_counter % 60 != 0:
        return

    # Convert ROS msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        colorFrame_cv = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        colorFrame_draw = colorFrame_cv.copy()  # Copy for drawing purposes
        depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
        draw_box = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Run YOLOv8 inference on the colorFrame
    results = model(colorFrame_cv)

    # Check interval between callback
    end = time.time()
    print(f"Time between frames: {end - start:.2f} seconds")
    start = end

    # Initialize list to store coordinates of boxes centre point
    realXZs = []

    for result in results:

        # Obtain class names the model can detect
        names = result.names
        print(f"names: {names}")
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh

            # Convert tensor to numpy ndarray
            xywh = xywh.to("cpu").detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # Skip if clsName is not red or blue ball
            if clsName not in ["red_ball", "blue_ball"] or conf < 50:
                continue

            # Obtain xy which is centre coords of the box
            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)

            # Extract the coordinates and dimensions of the bounding box
            x_center, y_center, width, height = xywh

            # Calculate the height of each section (one-third of the original height)
            section_height = height / 3

            # Calculate the y-coordinates for the centers of the new bounding boxes
            y_center1 = y_center - section_height
            y_center2 = y_center
            y_center3 = y_center + section_height

            # Create new bounding boxes for each section
            box1 = [x_center, y_center1, width, section_height]
            box2 = [x_center, y_center2, width, section_height]
            box3 = [x_center, y_center3, width, section_height]

            # print the box1,2,3 x_center, y, width and height
            print(f'box1 : {box1}, box2 : {box2}, box 3 : {box3}')

            # Calculate depth and real x-coordinate
            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame_cv)
            realXZs.append([real_x, depth, clsName])
            print(f"realx: {real_x}, depth: {depth}, clsName: {clsName}")

    # Update the silo matrix and draw bounding boxes around detected silos
    updateSiloMatrix(realXZs)

    # Publish the matrix to a ROS topic
    msg = Int32MultiArray(data=silo_matrix.flatten().tolist())
    pubSiloState.publish(msg)
    print(f"silo_matrix: {silo_matrix}")

    # Display the annotated frame with silo bounding boxes
    try:

        # Draw the new bounding boxes on the image
        #box 1 
        cv2.rectangle(colorFrame_draw, (int(x_center - width / 2), int(y_center1 - section_height / 2)), 
                        (int(x_center + width / 2), int(y_center1 + section_height / 2)), (0, 255, 0), 2)
        #box 2 
        cv2.rectangle(colorFrame_draw, (int(x_center - width / 2), int(y_center2 - section_height / 2)), 
                        (int(x_center + width / 2), int(y_center2 + section_height / 2)), (0, 255, 0), 2)
        #box 3
        cv2.rectangle(colorFrame_draw, (int(x_center - width / 2), int(y_center3 - section_height / 2)), 
                        (int(x_center + width / 2), int(y_center3 + section_height / 2)), (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Inference", colorFrame_draw)
        cv2.waitKey(1)
    except Exception as e:
        print(f"Failed to display frame: {e}")

    # Motor and Gripper control code (omitted for brevity)

def updateSiloMatrix(realXZs):
    global silo_matrix

    # Reset silo matrix to zeros
    silo_matrix = np.zeros((3, 5), dtype=int)

    for realXZ in realXZs:
        real_x, depth, clsName = realXZ
        if clsName == "red_ball":
            ball_value = 1
        elif clsName == "blue_ball":
            ball_value = 2
        else:
            ball_value = 0

        # Determine the section where the ball falls based on the real x-coordinate
        section_index = min(max(int((real_x + 2.5) // 2.5), 0), 4)  # 0-4
        silo_matrix[2][section_index] = ball_value  # Update the bottom row of the matrix

    return silo_matrix


def calcX(depth, x, colorFrame):
    resolution_w = colorFrame.shape[1]
    center_x = resolution_w // 2
    focal = 580
    real_x = (x - center_x) * (depth / focal)
    return int(real_x)

def getDepth(x, y, conf, clsName, depthFrame):
    window_size = 5
    pad_size = (window_size - 1) // 2
    padded_img = cv2.copyMakeBorder(depthFrame, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=0)
    pix_in_win = padded_img[y - pad_size : y + pad_size + 1, x - pad_size : x + pad_size + 1]
    depth = pix_in_win.max()
    print(f"Depth value at ({x}, {y}) is {depth}, cls: {clsName}, conf: {conf}")
    return int(depth)

if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/best.pt")
    gripperState = "o"
    gripperArmState = "forward"
    print("done init")
    pubCoords = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    pubSiloState = rospy.Publisher("silo_state", Int32MultiArray, queue_size=10)
    pubGripperControl = rospy.Publisher("gripperControl", GripperControl, queue_size=10)
    pubMotorControl = rospy.Publisher("motorControl", MotorControl, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([colorSub, depthSub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("done sub/pub, callback")
    rospy.spin()
