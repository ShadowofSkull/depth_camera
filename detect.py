#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import message_filters


def convertImg(data, model):
    bridge = CvBridge()
    try:
      frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    results[0].save(filename='./imgs/result.jpg')  # save to disk

    # Wait key can act as rate limiter, and arg in it is in ms
    # 33 ms for 30 frames per second
    # check if user wants to quit by pressing q (need to in focus display window and press q to work)
    if cv2.waitKey(33) & 0xFF == ord("q"):
      return
    
    
  


    # try:
    #   image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
    # image_message = bridge.imgmsg_to_cv2(img, encoding="passthrough")


if __name__ == "__main__":
    rospy.init_node("detect")
    model = YOLO("./models/yolov8m.pt")
    # The worst inference time should be set as the rate limiter
    rate = rospy.Rate(50) # ROS Rate at Hz (1Hz = 10ms)
    # There don't seem to be a way to limit rate from subscriber only publisher need to find which script handle color stream and change it, also can set queue_size for publisher prob put 1
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback=convertImg(model=model))
    ts = message_filters.TimeSynchronizer([sub], 1)
    ts.registerCallback(convertImg(model=model))
    # test if spin once exists if not remove
    while not rospy.is_shutdown():
        rospy.spin_once()  # Process messages one at a time
        rate.sleep()  # Sleep to maintain the desired rate
    # Check if this part loop
    print("loop")
    rospy.spin()
    print("after spin")