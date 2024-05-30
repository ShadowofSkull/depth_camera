#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image
from astra_camera.msg import CoordsMatrix
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

def callback(coords, depthFrame):
    coords = coords.coords
    
    try:
        frame = bridge.imgmsg_to_cv2(depthFrame, "passthrough")
    except CvBridgeError as e:
        print(e)


    for coord in coords:
        # Get x y to get depth value at the center of object
        x = coord.x
        y = coord.y
        conf = coord.conf
        cls = coord.cls

        print(x, y, conf, cls)
        depthVal = frame[y][x]
        count = 0
        print(f"outside loop: {depthVal}")
        # Shifting the centre point to the right to get something other than zero 5 times is the limit to prevent when the z value is actually zero
        # Index limit is also set to not have index error 
        while depthVal == 0 and count < 5 and x < 635 and y < 475 :
            x += 5
            y += 5
            count += 1
            depthVal = frame[y][x]
            print(depthVal)

        # Make a copy of it to make it writable
        # frame = frame.copy()
        # print(frame.shape)
        # To check position is correct
        # frame[y:y+5, x:x+5] = 10000
        # cv2.imshow("fr", frame)
        # cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("depth")
    bridge = CvBridge()
    # rate =
    print("done init")
    # pub = rospy.Publisher("depthValue", , queue_size=10)
    coordsSub = message_filters.Subscriber("/coords", CoordsMatrix)
    depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    print("subscribed")
    ts = message_filters.ApproximateTimeSynchronizer([coordsSub, depthSub], 10, 0.1, allow_headerless=True)
    print("sync subs")
    ts.registerCallback(callback)
    rospy.spin()
