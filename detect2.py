#!/usr/bin/env python3
import message_filters
import time
import torch
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

torch.cuda.set_device(0)

# Initialize global variables
frame_counter = 0

def callback(colorFrame, depthFrame):
    global frame_counter, start, model, pubCoords
    frame_counter += 1

    # Process every 5th frame
    if frame_counter % 5 != 0:
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
    depthCoords = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.to('cpu').detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # Obtain xy which is centre coords of
            x, y, w, h = xywh[0]
            x = int(x)
            y = int(y)
            coord = Coords()
            coord.x = x
            coord.y = y
            coord.conf = conf
            coord.cls = clsName
            depthCoords.append(coord)

            # Visualize the results on the frame (optional)
            annotated_frame = result.plot()
            # Draw a red dot at the centre of the box illustration purpose
            annotated_frame[y : y + 5, x : x + 5] = [0, 0, 255]

        # Prevent displaying error when no boxes are detected
        if boxes == None:
            continue

        # Display the annotated frame (optional)
        # Comment out the display of the inference to get values without hanging 
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(100)

    # Instead of processing on another node process depth here so the color and depth frame matches
    for coord in depthCoords:
        x = coord.x
        y = coord.y
        conf = coord.conf
        cls = coord.cls

        depthVal = depthFrame[y][x]
        # count = 0
        # while depthVal == 0 and count < 5 and x < 635 and y < 475:
        #     x += 5
        #     y += 5
        #     count += 1
        #     depthVal = depthFrame[y][x]
        print(f"Depth value at ({x}, {y}) is {depthVal}, cls: {cls}")

    # Publish xy to a topic so depth node can use it
    msg = CoordsMatrix()
    msg.coords = depthCoords
    pubCoords.publish(msg)

def calcXYZ(depthVal):
    # Placeholder for future calculations
    pass

if __name__ == "__main__":
    rospy.init_node("detect")
    start = time.time()
    model = YOLO("./models/yolov8m.pt")
    pubCoords = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([colorSub, depthSub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
