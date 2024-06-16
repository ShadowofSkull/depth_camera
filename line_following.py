import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Process the image to detect the line
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # method 2
    # Convert to grayscale
    # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # # Gaussian Blur to reduce noise
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # # Edge detection using Canny
    # edges = cv2.Canny(blur, 50, 150)

    # # Detect lines using HoughLinesP
    # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)

    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # Draw the contours
    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

    cv2.imshow("Line Following", cv_image)
    cv2.waitKey(1)


rospy.init_node("line_follower")
rospy.Subscriber("/camera/colorimage_raw", Image, image_callback)
rospy.spin()
