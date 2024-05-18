
"""
# Install necessary packages
pip3 install roboflow
pip install ultralytics==8.0.196
pip install IPython roboflow ultralytics==8.0.196


# if installing ultralytics doesnt work, try create a virtual environment 
python -m venv myenv
for windoes ; myenv\Scripts\activate
for mac ; source myenv/bin/activate
pip install IPython roboflow ultralytics==8.0.196
python3 yolo_detect.py

"""

from IPython import display
import glob
from IPython.display import Image, display as ipy_display
from roboflow import Roboflow
from ultralytics import YOLO
from PIL import Image as PILImage
import cv2

# Clear any previous outputs
display.clear_output()

# Check Ultralytics installation
import ultralytics
ultralytics.checks()

# Initialize Roboflow with API key
rf = Roboflow(api_key="JRqBrq8REBsf88PzUOWZ")

# Access Robocon ball dection project Roboflow
project = rf.workspace("draker-master-nybia").project("color-balls")
dataset = project.version(2).download("yolov8")

# Define paths and parameters
model_path = "yolov8m.pt"
data_yaml = f"{dataset.location}/data.yaml"
epochs = 20
imgsz = 640
best_model_path = "/content/runs/detect/train/weights/best.pt"
prediction_output_path = './content/runs/detect/predict2/*.png'

# Train the model
model = YOLO(model_path)
#model.train(data=data_yaml, epochs=epochs, imgsz=imgsz)
# Export data to onnx for 3x speed on cpu / TensorRT/engine for 5x on GPU
model.export(format='onnx')
model.export(format='engine')
# Validate the model
#model.val(data=data_yaml, model=best_model_path)

# Perform predictions
#results = model.predict(model=best_model_path, conf=0.5, data=data_yaml)
#results = model.predict(model=model_path, conf=0.5, data=data_yaml)
results = model("./balls.jpeg", save=True)


# Display prediction results
for image_path in glob.glob(prediction_output_path):
    img = PILImage.open(image_path)
    ipy_display(img)
    print("\n")