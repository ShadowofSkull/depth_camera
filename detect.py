#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
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

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # cv2.destroyAllWindows()
    
    
    
    # accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
    # results = model.predict(source="0")
    # results = model.predict(source=frame, show=True) # Display preds. Accepts all YOLO predict arguments


    # try:
    #   image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
    # image_message = bridge.imgmsg_to_cv2(img, encoding="passthrough")


if __name__ == "__main__":
    rospy.init_node("detect")


    img = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg)
    print("after sub")
    rospy.spin()

    