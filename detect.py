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
    
    for result in results:
       print(f"Result og img nparr: {len(result.orig_img)}\nShape: {result.orig_shape}")
       
       print(f"Speed: {result.speed}")
       boxes = result.boxes
       for box in boxes:
          xywh = box.xywh
          print(f"xy coord and width height: {xywh}\nclass of box: {box.cls}\nconfidence: {box.conf[0]}\n\n")
          # the array should contain value of every pixel of the image within the box
          npArr = result.orig_img
          x,y,w,h = xywh[0]
          centrePixelX = x + w/2
          centrePixelY = y + h/2
          centrePixel = npArr[int(centrePixelY)][int(centrePixelX)]
          print(f"centre pixel value: {centrePixel}\ncentre pixel x: {centrePixelX}\ncentre pixel y: {centrePixelY}\n")
    # For depth cam try reading from the topic depth frame pub to or else would have to save frame to folder and read from there
    # if frame nparray same size then can directly compare xy to get centre pixel depth val to get distance
      

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

    