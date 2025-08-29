# Superbot_yolo

ROS2(foxy version) wrapper for (https://github.com/ultralytics/ultralytics). This package enables you to perform object detection and segmentation.
This repository is a modified version of [yolov8-ros2](https://github.com/BAN2ARU/yolov8-ros2)

## Modifications
- Converted yolov8_node into pose_estimation with YOLOv11-pose support and additional logic for distance estimation and hand gesture detection.
- Replaced camera input: no longer using usb_cam and image_raw, now using OpenCV VideoCapture.
- Added yolo_follow to convert distance and position into linear and angular velocities.
- Introduced PersonInfo message to store distance and horizontal (x) position of the detected person

## Installation

### Prerequisites
- Ubuntu 20.04
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

### Install Requirement

```
cd ~/superbot_ws
pip install -r ./src/superbot_yolo/requirements.txt
colcon build
source ./install/setup.bash
```

## Usage

### Start the yolo_pose launch
- this will launch pose_estimation node and yolo_follow simultaneously

```bash
ros2 launch yolo_launch yolo_foxy.launch.py
```

#### Notes
- for **weight** : check in yolo_main/resource, the engine model is for jetson orin nano only, use your yoloV11 own model, or you can use the pytorch model (train4.pt), or you can convert it into your engine model or some else. Then just simply copy your  model path and replace the self.model
- train4 model include 4 class, which is 0: 'batu' (rock), 1: 'gunting' (scissor), 2: 'kertas' (paper) 3: 'person'
- In this project, an AprilTag from the tag36h11 family with ID 0 is used
- In this implementation, I use OpenCV’s VideoCapture instead of subscribing to image_raw from usb_cam, because running it simultaneously with navigation, localization, and RViz introduces a 5–7 second delay.
- If you still need to share image messages for other tasks, you can use v4l2_camera (or v4l2-ctl for configuration), which provides lower latency.
