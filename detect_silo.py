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
        # Obtain classes name model can detect
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

            depth = getDepth(x, y, conf, clsName, depthFrame)
            real_x = calcX(depth, x, colorFrame_cv)
            realXZs.append([real_x, depth, clsName])
            print(f"realx: {real_x}, depth: {depth}, clsName: {clsName}")

    # Update the silo matrix and draw bounding boxes around detected silos
    updateSiloMatrix(realXZs, colorFrame_draw)

    # Publish the matrix to a ROS topic
    msg = Int32MultiArray(data=silo_matrix.flatten().tolist())
    pubSiloState.publish(msg)
    print(f"silo_matrix: {silo_matrix}")

    # Display the annotated frame with silo bounding boxes
    try:
        for box in silo_boxes:
            cv2.rectangle(colorFrame_draw, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), thickness=2)
        cv2.imshow("YOLOv8 Inference", colorFrame_draw)
        cv2.waitKey(1)
    except Exception as e:
        print(f"Failed to display frame: {e}")

    # Motor and Gripper control code (omitted for brevity)

def updateSiloMatrix(realXZs, frame):
    global silo_matrix, silo_boxes
    # Clear the matrix and silo boxes
    silo_matrix.fill(0)
    silo_boxes = []

    # Determine the silo boundaries dynamically based on detected balls
    for x, z, clsName in realXZs:
        # Assuming each silo is 100 pixels wide and there are 5 silos
        silo_width = frame.shape[1] // 5

        # Determine the silo index based on x coordinate
        silo_idx = x // silo_width
        if silo_idx >= 5:
            continue  # Skip if somehow outside the frame (shouldn't happen)

        # Determine the row based on depth (z value)
        if z < 1000:
            row = 0
        elif z < 2000:
            row = 1
        else:
            row = 2

        # Set the matrix value based on the ball color
        if clsName == "red_ball":
            silo_matrix[row, silo_idx] = 1
        elif clsName == "blue_ball":
            silo_matrix[row, silo_idx] = 2

        # Calculate bounding box coordinates for the detected silo (traffic light style)
        box_x_min = int(silo_idx * silo_width)
        box_x_max = int((silo_idx + 1) * silo_width)
        box_y_min = 0
        box_y_max = frame.shape[0]
        silo_boxes.append((box_x_min, box_y_min, box_x_max, box_y_max))

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

