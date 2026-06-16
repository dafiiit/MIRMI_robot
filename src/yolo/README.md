# YOLOv8 Pose Model Deployment
This guide explains how to load and run inference using the trained YOLOv8 pose model (`yolo26n.pt`) to detect the docking station keypoints.
## 1. Install Dependencies
Install the official Ultralytics YOLO package:
```bash
pip install ultralytics opencv-python
```
## 2. Inference Script
Create a script `detect.py` to run inference and extract the 3D bounding box corners (8 keypoints):
```python
import cv2
from ultralytics import YOLO
# 1. Load the trained model
model = YOLO("yolo26n.pt")
# 2. Run inference (set confidence threshold as needed)
results = model("path/to/image.jpg", conf=0.25)
# 3. Parse and display results
for r in results:
    # Get bounding boxes (xyxy)
    if r.boxes is not None:
        boxes = r.boxes.xyxy.cpu().numpy()
        print("Detected boxes:", boxes)
        
    # Get 3D docking station corners (8 keypoints)
    if r.keypoints is not None and r.keypoints.xy is not None:
        keypoints = r.keypoints.xy.cpu().numpy() # Shape: (N, 8, 2)
        print("Docking station keypoints (corners):\n", keypoints)
```
## 3. Pretrained Base Model
* `yolov8n-pose.pt` is the standard pretrained COCO pose base used for training initialization.
* `yolo26n.pt` is the custom-trained model for the 3D docking station pose estimation.
