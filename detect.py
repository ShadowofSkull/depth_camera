#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO


def convertImg(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    model = YOLO("yolov8m.pt")
    print("init yolo")

    # Run YOLOv8 inference on the frame
    results = model(frame)
    msg = Int16MultiArray()
    depthCoords = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            xywh = box.xywh
            print(
                f"xy coord and width height: {xywh}\nclass of box: {box.cls}\nconfidence: {box.conf[0]}\n\n"
            )
            # the array should contain value of every pixel of the image within the box
            x, y, w, h = xywh[0]
            # x y is the centre pixel of the box
            x = int(x)
            y = int(y)
            depthCoords.append([x, y])
            # Visualize the results on the frame
            annotated_frame = result.plot()
            # annotated_frame[int(centrePixelY): int(centrePixelY) + 5, int(centrePixelX): int(centrePixelX)+5] = [0,0,255]
            # Draw a red dot at the centre of the box illustration purpose
            annotated_frame[y : y + 5, x : x + 5] = [0, 0, 255]
            # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(10)
        # Publish xy to a topic so depth node can use it
    pub = rospy.Publisher("chatter", Int16MultiArray, queue_size=10)
    msg.data = depthCoords
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("detect")
    try:
        img = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg)
    except rospy.ROSInterruptException:
        print("ERR")
    rospy.spin()
