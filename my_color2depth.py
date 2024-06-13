#!/usr/bin/env python3

import message_filters
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import TransformListener, Buffer
import math
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


tf_listener = None
tf_buffer = None


def callback(color_msg: Image):
    cv_bridge = CvBridge()
    color_img = cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")

    center_x = color_img.shape[1] / 2
    center_y = color_img.shape[0] / 2
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
    color_point.point.x = center_x
    color_point.point.y = center_y
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
    print(
        "from color {},{} to depth {},{}".format(center_x, center_y, depth_x, depth_y)
    )


def main():
    rospy.init_node("test_sync", anonymous=True)
    color_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)
    global tf_listener, tf_buffer
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.spin()


if __name__ == "__main__":
    main()
