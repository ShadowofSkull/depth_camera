import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def cb(msg):
    bridge = CvBridge()
    try:
        colorFrame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("test", colorFrame)
    cv2.waitKey(1)
    cv2.destroyAllWindows()

rospy.init_node("test")
sub = rospy.Subscriber("/camera/color/image_raw", Image, callback=cb)

rospy.spin()