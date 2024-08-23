# import cv2
# import numpy as np

# # https://medium.com/@achmad01/how-to-use-the-yolov8-model-with-opencv-onnx-inference-cd2e30c36ecf
# net = cv2.dnn.readNetFromONNX("./models/best.onnx")
# INPUT_WIDTH = 640
# INPUT_HEIGHT = 640

# image = cv2.imread('/path/to/your_image.jpg')
# blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
# net.setInput(blob)

# output = net.forward()
# output = output.transpose((0, 2, 1))

# # Extract output detection
# class_ids, confs, boxes = list(), list(), list()

# image_height, image_width, _ = image.shape
# x_factor = image_width / INPUT_WIDTH
# y_factor = image_height / INPUT_HEIGHT

# rows = preds[0].shape[0]

# for i in range(rows):
#     row = preds[0][i]
#     conf = row[4]
    
#     classes_score = row[4:]
#     _,_,_, max_idx = cv2.minMaxLoc(classes_score)
#     class_id = max_idx[1]
#     if (classes_score[class_id] > .25):
#         confs.append(conf)
#         label = int(class_id)
#         class_ids.append(label)
        
#         #extract boxes
#         x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
#         left = int((x - 0.5 * w) * x_factor)
#         top = int((y - 0.5 * h) * y_factor)
#         width = int(w * x_factor)
#         height = int(h * y_factor)
#         box = np.array([left, top, width, height])
#         boxes.append(box)
        
# r_class_ids, r_confs, r_boxes = list(), list(), list()

# indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.25, 0.45) 
# for i in indexes:
#     r_class_ids.append(class_ids[i])
#     r_confs.append(confs[i])
#     r_boxes.append(boxes[i])


# for i in indexes:
#     box = boxes[i]
#     left = box[0]
#     top = box[1]
#     width = box[2]
#     height = box[3]
#     conf = confs[i]
    
#     cv2.rectangle(image, (left, top), (left + width, top + height), (0,255,0), 3)

# cv2.imshow("test", image)
import numpy as np
mat = [[[112, "c"],[293, "a"], [112, "b"]], [[112, "c"],[100, "a"], [112, "b"]]]
# for i in mat:
#     if i[0] 
def test(e):
    return e[0]
for i in mat:
    i.sort(key=test)
# mat.sort(key=test)
print(mat)