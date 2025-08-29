# 🛒 Semi-Autonomous Mobile Robot for Supermarket Shopping Efficiency

## 📌 Abstract
The rapid advancement of robotics has driven the emergence of innovative solutions to enhance the convenience and efficiency of supermarket shopping. This study presents the design of a prototype semi-autonomous mobile supermarket robot that integrates autonomous navigation, precise motor control, and intuitive human–robot interaction based on human following and hand gestures. The robot employs a configuration of two omnidirectional wheels and a caster wheel for balance, driven by PG45 DC motors controlled by an ESP32 microcontroller with a BTS7960 driver. A PI control system with parameters Kp = 0.1 and Ki = 0.004 maintains motor speed with a steady-state error below 5% and without significant overshoot, both under no-load and load conditions up to 30 kg. For navigation, the robot is equipped with an RPLiDAR A2 sensor, odometry, and mapping based on the SLAM Toolbox. Path planning is implemented using the A-Star algorithm and the Dynamic Window Approach (DWA). Experimental results demonstrate that the robot successfully completes multi-point navigation and avoids both static and dynamic obstacles, with an average response time of 3.51–6.52 seconds. Beyond navigation, the robot enables natural human interaction through an RGB camera processed with the YOLOv11-Pose deep learning model, achieving an F1-score above 0.92 and inference speed of 14 FPS. Distance estimation is performed using triangulation of head–neck keypoints, yielding an average accuracy of MAPE 5.3%. Hand gesture detection demonstrates high reliability with confidence scores above 0.83 at distances of 1.5–5 meters, allowing users to flexibly activate human-following mode or stop the robot. Tests conducted in a supermarket-like simulation environment confirm that the robot can stably follow users, maneuver in narrow aisles, and adaptively switch between operating modes. By combining precise motor control, SLAM-based navigation, and gesture-based interaction, this supermarket robot prototype shows strong potential as an intelligent solution to support a more ergonomic, efficient, and adaptive shopping experience. The perception system includes:

- **RGB Camera** → YOLOv11-Pose for human detection, pose estimation, and hand gesture recognition.
- **LiDAR (RPLiDAR A2)** → 2D mapping and obstacle detection.
- **Odometry (wheel encoders)** → position estimation.
- **Load Cell** → payload weight monitoring.

The navigation system uses **SLAM Toolbox** for mapping, **AMCL** for localization, and **Nav2** (A* for global path, DWA for local planner).

---

## ⚙️ System Architecture
The system is divided into modules connected via ROS 2 topics and services:

- **Perception**: RGB Camera + YOLOv11-Pose, LiDAR, Odometry.
- **Control**: Differential drive with omnidirectional wheels, ESP32 motor control, Load cell feedback.
- **Navigation**: SLAM Toolbox, AMCL, Nav2 (A*, DWA).
- **GUI**: Tkinter-based interface for product categories, teleoperation, and visualization in RViz.

---

## 🖼️ System 

### 🔧 Wheel System
| ![Wheel Design](https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/wheels.png) | ![Alternative Wheel Design](https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/wheels2.png) |
|------------------------------------------------------------|----------------------------------------------------------------|

### 🗺️ Mapping & SLAM
| <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/map.png" width="250"/> | <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/rviz.png" width="600"/> |
|------------------------------------------------------------|----------------------------------------------------------------|

### 🖥️ GUI Interface
| <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/GUI.png" width="400"/> | <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/imageanotation.png" width="400"/> |
|------------------------------------------------------------|----------------------------------------------------------------|

### 👀 Human Detection & Gesture
| <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/humanfollowing.png" width="400"/> | <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/coordinatepoint.png" width="400"/> |
|-------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|


---

## 🎯 Methods

### 1. Hardware
- **Jetson Orin Nano / Xavier NX** → AI inference & high-level computation.
- **ESP32** → Low-level motor control, encoder feedback, load cell data.
- **DC Motors + Omnidirectional wheels** → Maneuverability in narrow aisles.
- **RPLiDAR A2** → 360° scanning.
- **RGB Camera** → Human & gesture detection.
- **Load Cell** → Payload monitoring.

### 2. Perception (YOLOv11-Pose)
- Detects **human**, **keypoints (head–neck)** for distance estimation, and **hand gestures**.
- Custom dataset: 6,650 images, classes = {person, rock, paper, scissors}.
- Training with transfer learning → converted to TensorRT engine for Jetson.
- FPS ≈ 14, F1-score ≈ 0.92.

### 3. Human Following & Gesture Control
- Hybrid tracking with **AprilTag + YOLOv11-Pose + Kalman Filter**.
- Distance estimation via vertical keypoint distance (head–neck triangulation).
- Hand gestures mapped to robot commands:
  - **Rock (✊)** → STOP
  - **Paper (✋)** → HUMAN FOLLOW
  - **Scissors (✌️)** → GO

### 4. Navigation
- **Mapping**: SLAM Toolbox (multi-session mapping, pose graph optimization).
- **Localization**: AMCL with adaptive particle filter.
- **Global Planner**: A* (efficient path planning).
- **Local Planner**: DWA (dynamic obstacle avoidance).

### 5. Load Cell Integration
- Measures payload weight in real-time.
- Publishes weight to GUI and ROS 2 topics.
- GUI indicator: color-coded status (safe, medium, overload).

---

## ▶️ Usage

### Build and Setup
```bash
# Clean build
rm -rf build/ install/ log/

# Build
colcon build --packages-select supermarket_ws

# Source
source install/setup.bash
```

### Run SLAM & Navigation
```bash
# Mapping with SLAM Toolbox
ros2 launch supermarket_ws slam_toolbox.launch.py

# Localization + Nav2
ros2 launch supermarket_ws localization.launch.py
ros2 launch supermarket_ws navigation.launch.py
```

### Run YOLOv11 Inference
```bash
ros2 run supermarket_robot yolo_inference.py
```

### GUI Control
```bash
ros2 run supermarket_robot gui_control.py
```

---

## 📍 Notes
- Nav2 tested with A* (global) + DWA (local). TEB considered but computationally heavier.
- Load cell integrated into GUI with reset function.
- SLAM Toolbox preferred over GMapping/Cartographer due to efficiency.

---

## 🤖 System Block Diagram

### 👀 System Block Diagram Robot
| <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/blockdiagram.png" width="400"/> |
|---------------------------------------------------------------------------------------------------------------|

---

## 🔧 Installation & Setup
```bash
sudo apt install ros-foxy-tf2-tools
sudo apt install ros-foxy-robot-localization
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3*
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-slam-toolbox
sudo apt install ros-foxy-twist-mux
sudo apt install joystick jstest-gtk evtest

pip install rplidar
pip install serial
```

- Install Arduino IDE & ESP32 package v2
- Reference: [Differential drive robot using ROS2 and ESP32](https://www.reddit.com/r/differential-drive-robot-using-ros2-and-esp32-aae289)
- Micro-ROS Serial Tutorial: [YouTube](https://www.youtube.com/watch?v=cLlK7uqFC2A)
- Motor controller demo: [YouTube](https://www.youtube.com/watch?v=-PCuDnpgiew)
- Hackster tutorial: [Hackster.io/amal-shaji](https://www.hackster.io/amal-shaji)

---

## 📖 References
- YOLOv11-Pose human detection & gesture recognition
- ROS 2 Navigation Stack (Nav2)
- SLAM Toolbox for mapping
- AMCL localization
- ESP32 differential drive controller integration
