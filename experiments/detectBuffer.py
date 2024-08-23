# YOLO can activate stream for a few data types which create a generator which will be memory efficient potentially fixing the lagging issue, refer https://docs.ultralytics.com/modes/predict/#key-features-of-predict-mode:~:text=Use%20stream%3DTrue,of%2Dmemory%20issues.

# By creating a directory and storing images into it and then deleting them after some time and making the model read from the directory which allow stream

import cv2
import os
from ultralytics import YOLO
from pathlib import Path
import time


def write_buffer(buffer_path, buffer_size = 5):
    # Create buffer directory if not exist
    Path(buffer_path).mkdir(parents=True, exist_ok=True)
    print("Buffer folder created")
    i = 0
    while i < buffer_size:
        # Frame in real app should use the camera stream
        frame = cv2.imread(f"./imgs/balls{i + 1}.jpg")
        filename = f"img{i}.png"

        # Store it in folder
        try:
            cv2.imwrite(buffer_path + filename, frame)
        except:
            print(f"Failed to store image {filename}")
        print(f"Image {filename} stored in buffer")
        i += 1


def inference(model, buffer_path):
    # Inference with model
    results = model(buffer_path, stream=True)
    return results

        
def post_process(results):
    # Create results directory if not exist
    Path("./results").mkdir(parents=True, exist_ok=True)
    # Loop the results and display the results
    i = 0
    for result in results:
        annotated_frame = result.plot()  # annotates results on image
        try:
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            cv2.waitKey(20)
        except:
            print(f"Failed to display result {i}")

        try:
            result.save(filename=f'./results/result{i}.jpg')  # save to disk
        except:
            print(f"Failed to save result {i}")
        i += 1
    
    cv2.destroyAllWindows()
        

def clear_buffer(buffer_path, buffer_size = 5):
    # Remove previously inferenced img
    for i in range(buffer_size):
        try:
            os.remove(buffer_path + f"img{i}.png")
        except:
            print(f"Failed to remove file, img{i}.png")


if __name__ == "__main__":
    # Processing in batch seems to reduce inference time
    model = YOLO("./models/yolov8m.pt")
    print("Model loaded")
    buffer_size = 5
    buffer_path =  "./buffer/"
    while True:
        write_buffer(buffer_path, buffer_size)
        print("Buffer filled")

        results = inference(model, buffer_path)
        print("Inference done")

        post_process(results)
        print("Results displayed")

        clear_buffer(buffer_path, buffer_size)
        print("Buffer cleared")
        # Can be replace with rospy rate rate sleep (use rate sleep instead of spin)
        time.sleep(5)