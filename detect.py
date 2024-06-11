#!/usr/bin/env python3
import message_filters
import time
import rospy
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix, Coords
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO


def callback(colorFrame, depthFrame):
    # Convert ros msg to cv nparray that's suitable for model
    bridge = CvBridge()
    try:
        colorFrame = bridge.imgmsg_to_cv2(colorFrame, "bgr8")
        depthFrame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")

    except CvBridgeError as e:
        print(e)
    print(type(depthFrame))
    # Run YOLOv8 inference on the colorFrame
    results = model(colorFrame)
    # Check interval between callback
    end = time.time()
    print(end - start)
    # Initialise list to store coordinates of boxes centre point
    depthCoords = []
    for result in results:
        # Obtain classes name model can detect
        names = result.names
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            # Convert tensor to numpy ndarray
            xywh = xywh.numpy()
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
            # Draw a red dot at the centre of the box illustration purpose
            annotated_frame[y : y + 5, x : x + 5] = [0, 0, 255]
        # Prevent displaying error when no boxes are detected
        if boxes == None:
            continue
        # Display the annotated frame
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(1)
    # Instead of processing on another node process depth here so the color and depth frame matches
    for coord in depthCoords:
        # Get x y to get depth value at the center of object
        x = coord.x
        y = coord.y
        conf = coord.conf
        cls = coord.cls

        print(x, y, conf, cls)
        # depthVal means distance from camera to object in mm
        depthVal = depthFrame[y][x]
        print(f"outside loop: {depthVal}")

        count = 0
        # Shifting the centre point to the right to get something other than zero 5 times is the limit to prevent when the z value is actually zero
        # Index limit is also set to not have index error
        while depthVal == 0 and count < 5 and x < 635 and y < 475:
            x += 5
            y += 5
            count += 1
            depthVal = depthFrame[y][x]
            print(depthVal)

    # Publish xy to a topic so depth node can use it
    msg.coords = depthCoords
    # print(msg.coords)
    pubCoords.publish(msg)


def calcXYZ(depthVal):
    # how to get xyz https://3dclub.orbbec3d.com/t/mapping-units-of-depth-image-and-meters/1600/5#:~:text=Apr%202018-,Once,-you%20have%20the
    # cam spec https://shop.orbbec3d.com/Astra
    pass


if __name__ == "__main__":
    rospy.init_node("detect")
    msg = CoordsMatrix()
    model = YOLO("./models/yolov8m.pt")
    # Detect every 6 frames
    # rate = rospy.Rate(0.2)
    print("done init")
    pubCoords = rospy.Publisher("coords", CoordsMatrix, queue_size=10)
    colorSub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer(
        [colorSub, depthSub], 10, 0.1, allow_headerless=True
    )
    ts.registerCallback(callback)

    start = time.time()
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rate.sleep()
