import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    # Convert the image to the BGR format
    bgr_color = np.uint8([[[249, 240, 246]]])  # BGR representation of Pantone 663C
    hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)
    
    # H (Hue): 0 to 179 (instead of 0 to 360 degrees)
    # S (Saturation): 0 to 255 (instead of 0 to 100%)
    # V (Value): 0 to 255 (instead of 0 to 100%)
    # hsv_color[0][0][0] represents the Hue component

    # Define the HSV range for Pantone 663C
    lower_bound = np.array([hsv_color[0][0][0] - 10, 50, 50])
    upper_bound = np.array([hsv_color[0][0][0] + 10, 255, 255])

    # Convert the image to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Create a mask for the specified color range
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contours on the original image
    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

    # Display the image
    cv2.imshow("Line Following - Pantone 663C", cv_image)
    cv2.waitKey(1)

# Initialize the ROS node
rospy.init_node('line_follower')
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
