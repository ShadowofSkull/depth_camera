#!/usr/bin/env python3
import message_filters
import time
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ultralytics import YOLO
from tf2_ros import TransformListener, Buffer
import math
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import torch

torch.cuda.set_device(0)


def callback(colorFrame, depthFrame):
    global lastColorFrame, lastDepthFrame
    # Convert ros msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        lastColorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        lastDepthFrame = bridge.imgmsg_to_cv2(depthFrame, "16UC1")
        print("callbvack")
    except CvBridgeError as e:
        print(e)


def inference():
    global lastDepthFrame, lastColorFrame
    if lastColorFrame == None or lastDepthFrame == None:
        return
    # Run YOLOv8 inference on the colorFrame
    results = model(lastColorFrame)

    # Initialise list to store coordinates of boxes centre point
    depthCoords = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            # Move xywh from gpu to cpu
            xywh = xywh.to('cpu').detach().numpy().copy()
            conf = int(box.conf[0] * 100)
            cls = int(box.cls[0])
            clsName = names[cls]

            # Obtain xy which is centre coords of
            x, y, w, h = xywh[0]
            # x y is the centre pixel of the box
            x = int(x)
            y = int(y)
            coord = Coords()
            coord.x = x
            coord.y = y
            coord.conf = conf
            coord.cls = clsName
            # print(x, y, conf, clsName)
            depthCoords.append(coord)
            # Visualize the results on the frame
            annotated_frame = result.plot()

        # Prevent displaying error when no boxes are detected
        # if boxes == None:
        #     continue
        # Display the annotated frame
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(1)
    # Instead of processing on another node process depth here so the color and depth frame matches
    minDistance, real_x = getClosestBall(depthCoords)
    rate = rospy.Rate(0.2)
    rate.sleep()


def getClosestBall(depthCoords):
    global lastDepthFrame
    minDistance = None
    for coord in depthCoords:
        # Get x y to get depth value at the center of object
        x = coord.x
        y = coord.y
        conf = coord.conf
        cls = coord.cls
        # check topic header for frame id
        try:
            trans = tf_buffer.lookup_transform(
                "camera_color_frame", "camera_depth_frame", rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.loginfo(e)
            return

        # Create a point in the depth camera's coordinate frame
        color_point = PointStamped()
        color_point.header.frame_id = "camera_color_frame"
        color_point.header.stamp = rospy.Time.now()
        color_point.point.x = x
        color_point.point.y = y
        # color_point.point.z = color_img[int(center_y), int(center_x)]

        # Now we transform the point from the depth camera's coordinate frame to the color camera's coordinate frame
        try:
            depth_point = do_transform_point(color_point, trans)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.loginfo(e)
            return

        # The point's position in the color frame is now stored in color_point.point
        depth_x = depth_point.point.x
        depth_y = depth_point.point.y
        print("from color {},{} to depth {},{}".format(x, y, depth_x, depth_y))

        print(f"conf:{conf}, cls:{cls}")
        # depthVal means distance from camera to object in mm
        depthVal = lastDepthFrame[depth_y][depth_x]
        print(f"depth val: {depthVal}")
        if depthVal == 0:
            print("frame hole wait for next inference")
            continue
        if not (cls == "ball" and conf > 50):
            print("not ball")
            continue

        distance, real_x = calcDistanceAndX(depthVal, x)
        if distance < minDistance or minDistance == None:
            minDistance = distance
            res = (minDistance, real_x)
    return res


def calcDistanceAndX(depth, x):
    fov_w = 49.5
    resolution_w = lastColorFrame.shape[1]
    center_x = resolution_w / 2
    if x <= center_x:
        theta_x = fov_w / resolution_w * (center_x - x)
    theta_x = fov_w / resolution_w * (x - center_x)
    real_x = math.tan(theta_x) * depth
    print(f"real x: {real_x}mm")
    distance = math.sqrt(real_x**2 + depth**2)
    return distance, real_x


if __name__ == "__main__":
    rospy.init_node("detect")
    msg = CoordsMatrix()
    model = YOLO("./models/yolov8m.pt")
    lastColorFrame = None
    lastDepthFrame = None
    print("done init")
    # pubCoords = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    # ts = message_filters.ApproximateTimeSynchronizer(
    #     [colorSub, depthSub], 10, 0.1, allow_headerless=True
    # )
    ts = message_filters.TimeSynchronizer([colorSub, depthSub], 10)
    ts.registerCallback(callback)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    # Detect every 6 frames

    start = time.time()
    inference()

    rospy.spin()
