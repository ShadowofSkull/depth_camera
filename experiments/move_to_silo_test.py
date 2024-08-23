#!/usr/bin/env python3

import rospy
import message_filters
import torch
from sensor_msgs.msg import Image
from astra_camera.msg import SiloMatrix, MotorControl, GripperControl
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np
import time 

torch.cuda.set_device(0)
frame_counter = 0

def callback(colorFrame, depthFrame):
    global frame_counter, start, model, pubMotorControl
    frame_counter += 1

    # Process every 5th frame
    if frame_counter % 60 != 0:
        return

    bridge = CvBridge()
    try:
        colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    results = model(colorFrame)
    end = time.time()
    start = end

    silos = []

    for result in results:
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            xywh = xywh.to("cpu").detach().numpy().copy()
            cls = int(box.cls[0])
            clsName = names[cls]

            if clsName == "Silo":
                x, y, w, h = xywh[0]
                x, y = int(x), int(y)
                depth = getDepth(x, y, depthFrame)
                silos.append((x, y, depth))

    if silos:
        x, y, depth = silos[0]  # Use the first detected silo
        move_towards_silo(x, depth, colorFrame.shape[1])

def getDepth(x, y, depthFrame):
    window_size = 5
    pad_size = (window_size - 1) // 2
    padded_img = cv2.copyMakeBorder(depthFrame, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=0)
    pix_in_win = padded_img[y - pad_size: y + pad_size + 1, x - pad_size: x + pad_size + 1]
    depth = np.median(pix_in_win)
    return int(depth)

def move_towards_silo(x, depth, frame_width):
    target_x = frame_width // 2
    error_x = target_x - x
    k_p = 0.01  # Proportional gain for steering
    speed = k_p * error_x

    target_depth_threshold = 20  # Distance threshold to the target
    if depth > target_depth_threshold:
        forward_speed = 0.5  # Move forward
    elif depth < target_depth_threshold:
        forward_speed = -0.5  # Move backward
    else:
        forward_speed = 0  # Stop

    motor1_speed = forward_speed + speed
    motor2_speed = forward_speed + speed
    motor3_speed = forward_speed - speed
    motor4_speed = forward_speed - speed

    motor_control_msg = MotorControl()
    motor_control_msg.motor1_speed = motor1_speed
    motor_control_msg.motor2_speed = motor2_speed
    motor_control_msg.motor3_speed = motor3_speed
    motor_control_msg.motor4_speed = motor4_speed

    pubMotorControl.publish(motor_control_msg)

if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/best.pt")

    pubMotorControl = rospy.Publisher("motor_control", MotorControl, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([colorSub, depthSub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

