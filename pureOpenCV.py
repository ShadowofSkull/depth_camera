#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def convertImg(data):
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
    # image_message = bridge.imgmsg_to_cv2(img, encoding="passthrough")


if __name__ == "__main__":
    rospy.init_node("detect")

    img = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg)
    rospy.loginfo("HELefao")
    rospy.spin()