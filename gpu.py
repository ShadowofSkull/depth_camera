import torch
import torchvision
from ultralytics import YOLO
from torchvision import transforms


# torch.cuda.set_device(0)
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# model = YOLO('/your/model/path/', task='detect')
# model.to(device=device)
# Check for CUDA device and set it
print(torch.__version__)
print(torchvision.__version__)
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0))
print(torch.cuda.device_count())

# Explicitly set pytorch to use the GPU
torch.set_default_tensor_type('torch.cuda.FloatTensor')
device = 'cuda' if torch.cuda.is_available() else 'cpu'
# Explicitly move model into gpu
model = YOLO("./models/yolov8m.pt").to(device)
# image is image from cv2
image = torch.from_numpy(image).to(device)  # Ensure your image tensor is also on GPU

# Ensure that the input image is correctly transformed into a tensor and moved to the GPU. This often involves normalizing and unsqueezing the image tensor before passing it to the model
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])
image = transform(image).unsqueeze(0).to(device)  # Add batch dimension and send to GPU

# Now run inference or training

