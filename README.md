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

### 🖥️ GUI Interface & Gesture
| <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/GUI.png" width="400"/> | <img src="https://github.com/ainnajib2002/Supermarket_Robot/blob/main/README/imageanotation.png" width="400"/> |
|------------------------------------------------------------|----------------------------------------------------------------|

### 👀 Human Detection 
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

[1] C. Thompson et al., "Changes to household food shopping practices during the COVID-19 restrictions: Evidence from the East of England," *Heal. Place*, vol. 78, no. July, p. 102906, 2022, doi: 10.1016/j.healthplace.2022.102906.

[2] S. Wang, Y. Ye, B. Ning, J. H. Cheah, and X. J. Lim, "Why Do Some Consumers Still Prefer In-Store Shopping? An Exploration of Online Shopping Cart Abandonment Behavior," *Front. Psychol.*, vol. 12, no. January, pp. 1–14, 2022, doi: 10.3389/fpsyg.2021.829696.

[3] H. Y. Purwantono, A. A. S. Gunawan, H. Tolle, M. Attamimi, and W. Budiharto, "A literature review: Feasibility Study of technology to improve shopping experience," *Procedia Comput. Sci.*, vol. 179, pp. 468–479, 2021, doi: 10.1016/j.procs.2021.01.030.

[4] D. L. M. Vikash Ranjan, Shafaq Akhtar, "Study on consumer satisfaction and loyalty towards," *Int. J. Adv. Multidiscip. Res.*, vol. 10, no. 1, pp. 89–99, 2023, doi: http://dx.doi.org/10.22192/ijamr.2023.10.01.009.

[5] A. Ramzan, R. Mustafa, Z. Rehman, S. Suleman, M. Noor, and R. Bibi, "Robotrolley: Customer Following Trolley (CFT)," *2023 17th Int. Conf. Open Source Syst. Technol. ICOSST 2023 - Proc.*, pp. 1–6, 2023, doi: 10.1109/ICOSST60641.2023.10414202.

[6] T. H. Tsai and C. H. Yao, "A robust tracking algorithm for a human-following mobile robot," *IET Image Process.*, vol. 15, no. 3, pp. 786–796, 2021, doi: 10.1049/ipr2.12062.

[7] G. C. Pătru, A. I. Pîrvan, D. Rosner, and R. V. Rughiniș, "Fiducial Marker Systems Overview and Empirical Analysis of Aruco, Apriltag and Cctag," *UPB Sci. Bull. Ser. C Electr. Eng. Comput. Sci.*, vol. 85, no. 2, pp. 49–62, 2023.

[8] D. Penaloza-Aponte et al., "Automated entrance monitoring to investigate honey bee foraging trips using open-source wireless platform and fiducial tags," *HardwareX*, vol. 20, no. November, p. e00609, 2024, doi: 10.1016/j.ohx.2024.e00609.

[9] A. Zachariae, F. Plahl, Y. Tang, I. Mamaev, B. Hein, and C. Wurll, "Human-robot interactions in autonomous hospital transports," *Rob. Auton. Syst.*, vol. 179, no. July, p. 104755, 2024, doi: 10.1016/j.robot.2024.104755.

[10] N. Reddy Kavya Sree, B. Anbarasu, and M. Luhitha, "Integration of LIDAR and Ultrasonic sensor for MAV collision avoidance," *ESIC 2025 - 5th Int. Conf. Emerg. Syst. Intell. Comput. Proc.*, pp. 832–836, 2025, doi: 10.1109/ESIC64052.2025.10962656.

[11] H. V. Pinnamaraju, P. R. Kapu, A. N. Juturu, and B. Anbarasu, "Distance Estimation for Collision Avoidance of Micro Aerial Vehicles using LiDAR Sensor," *Int. Conf. Autom. Comput. Renew. Syst. ICACRS 2022 - Proc.*, no. ICACRS, pp. 157–161, 2022, doi: 10.1109/ICACRS55517.2022.10029233.

[12] D. P. Suwandi, I. K. Wibowo, and M. M. Bachtiar, "Estimation of Ball Position Using Depth Camera for Middle Size Goalkeeper Robot," *IES 2022 - 2022 Int. Electron. Symp. Energy Dev. Clim. Chang. Solut. Clean Energy Transition, Proceeding*, pp. 374–379, 2022, doi: 10.1109/IES55876.2022.9888575.

[13] A. Mingozzi, A. Conti, F. Aleotti, M. Poggi, and S. Mattoccia, "Monitoring Social Distancing With Single Image Depth Estimation," *IEEE Trans. Emerg. Top. Comput. Intell.*, vol. 6, no. 6, pp. 1290–1301, 2022, doi: 10.1109/TETCI.2022.3171769.

[14] H. Herdianto, P. Sihombing, F. Fahmi, and T. Tulus, "Improving Object Distance Measurement Based on Mono Camera Using Magnification and Yolo Methods," *Int. Conf. Control Autom. Electron. Robot. Internet Things, Artif. Intell. CERIA 2024*, pp. 1–6, 2024, doi: 10.1109/CERIA64726.2024.10914711.

[15] Y. D. T. T. Wicaksana, A. Hendriawan, and N. Tamami, "Human Target Distance Estimation System Using Mono-camera On Human-Following Mobile Robot," *IES 2022 - 2022 Int. Electron. Symp. Energy Dev. Clim. Chang. Solut. Clean Energy Transition, Proceeding*, pp. 349–354, 2022, doi: 10.1109/IES55876.2022.9888331.

[16] T. Alhmiedat et al., "A SLAM-Based Localization and Navigation System for Social Robots: The Pepper Robot Case," *Machines*, vol. 11, no. 2, pp. 1–17, 2023, doi: 10.3390/machines11020158.

[17] G. Zhang, J. Yin, P. Deng, Y. Sun, L. Zhou, and K. Zhang, "Achieving Adaptive Visual Multi-Object Tracking with Unscented Kalman Filter," *Sensors*, vol. 22, no. 23, pp. 1–18, 2022, doi: 10.3390/s22239106.

[18] A. F. Rasheed and M. Zarkoosh, "YOLOv11 optimization for efficient resource utilization," *J. Supercomput.*, vol. 81, no. 9, 2025, doi: 10.1007/s11227-025-07520-3.

[19] L. Liu, A. Kurban, and Y. Liu, "Improved YOLOv11pose for Posture Estimation of Xinjiang Bactrian Camels," *Int. J. Adv. Comput. Sci. Appl.*, vol. 15, no. 12, pp. 364–371, 2024, doi: 10.14569/IJACSA.2024.0151239.

[20] M. Petković and I. Vujović, "Distance Estimation Approach for Maritime Traffic Surveillance Using Instance Segmentation," *J. Mar. Sci. Eng.*, vol. 12, no. 1, 2024, doi: 10.3390/jmse12010078.

[21] N. Herbaz, H. El Idrissi, and A. Badri, "Deep Learning Empowered Hand Gesture Recognition: using YOLO Techniques," *Proc. - SITA 2023 2023 14th Int. Conf. Intell. Syst. Theor. Appl.*, pp. 1–7, 2023, doi: 10.1109/SITA60746.2023.10373734.

[22] L. Zhou, C. Du, Z. Sun, T. L. Lam, and Y. Xu, "Long-Range Hand Gesture Recognition via Attention-based SSD Network," *Proc. - IEEE Int. Conf. Robot. Autom.*, vol. 2021-May, pp. 1832–1838, 2021, doi: 10.1109/ICRA48506.2021.9561189.

[23] L. Liu, D. Du, Y. Sun, and Y. Li, "SFMW-YOLO: A lightweight metal casting surface defect detection method based on modified YOLOv8s," *Expert Syst. Appl.*, vol. 287, no. May, p. 128170, 2025, doi: 10.1016/j.eswa.2025.128170.

[24] M. D. Islam et al., "Towards real-time weed detection and segmentation with lightweight CNN models on edge devices," *Comput. Electron. Agric.*, vol. 237, no. PB, p. 110600, 2025, doi: 10.1016/j.compag.2025.110600.

[25] S. Salimi, J. P. Queralta, and T. Westerlund, "Hyperledger Fabric Blockchain and ROS 2 Integration for Autonomous Mobile Robots," *2023 IEEE/SICE Int. Symp. Syst. Integr. SII 2023*, no. i, 2023, doi: 10.1109/SII55687.2023.10039326.

[26] P. A. Gutierrez-Flores and R. Bachmayer, "Concept development of a modular system for marine applications using ROS2 and micro-ROS," *2022 IEEE/OES Auton. Underw. Veh. Symp. AUV 2022*, 2022, doi: 10.1109/AUV53081.2022.9965867.

[27] H. Li et al., "Optimizing edge-enabled system for detecting green passion fruits in complex natural orchards using lightweight deep learning model," *Comput. Electron. Agric.*, vol. 234, no. March, p. 110269, 2025, doi: 10.1016/j.compag.2025.110269.

[28] S. Pan et al., "A lightweight robust RGB-T object tracker based on Jitter Factor and associated Kalman filter," *Inf. Fusion*, vol. 117, no. November 2024, p. 102842, 2025, doi: 10.1016/j.inffus.2024.102842.

[29] A. Kumar, R. Vohra, R. Jain, M. Li, C. Gan, and D. K. Jain, "Correlation filter based single object tracking: A review," *Inf. Fusion*, vol. 112, no. July, p. 102562, 2024, doi: 10.1016/j.inffus.2024.102562.

[30] S. Schramm, J. Rangel, D. A. Salazar, R. Schmoll, and A. Kroll, "Target Analysis for the Multispectral Geometric Calibration of Cameras in Visual and Infrared Spectral Range," *IEEE Sens. J.*, vol. 21, no. 2, pp. 2159–2168, 2021, doi: 10.1109/JSEN.2020.3019959.

[31] C. L. Shih, W. C. Huang, I. T. Anggraini, Y. Xiao, N. Funabiki, and C. P. Fan, "Performance Comparison between OpenPose and TRT-Pose for Self-Practice Yoga on Embedded GPU Platform," *2023 IEEE Int. Conf. Consum. Electron. ICCE-Asia 2023*, pp. 1–4, 2023, doi: 10.1109/ICCE-Asia59966.2023.10326363.

[32] J. Chen, S. Bai, G. Wan, Y. Li, J. Li, and Z. Cheng, "Research on defect detection method of automotive running lights based on keypoint identification," *Proc. 36th Chinese Control Decis. Conf. CCDC 2024*, pp. 5220–5225, 2024, doi: 10.1109/CCDC62350.2024.10588328.

[33] R. P. Duarte, C. A. Cunha, and J. C. Cardoso, "Automatic Camera Calibration Using a Single Image to extract Intrinsic and Extrinsic Parameters," *Orig. Res. Pap. Int. J. Intell. Syst. Appl. Eng. IJISAE*, vol. 2024, no. 3, pp. 1766–1778, 2024, [Online]. Available: www.ijisae.org

[34] T. H. Tsai and C. L. Lee, "Equipped with Monocular Depth Estimation and Intelligent Wake-Up Vision Based Tracking System for a Human-Following Mobile Robot," *2024 IEEE Int. Conf. Multimed. Expo Work. ICMEW 2024*, pp. 1–2, 2024, doi: 10.1109/ICMEW63481.2024.10645403.

[35] C. L. Yong, B. Hoe Kwan, D. W. K. Ng, and H. Seng Sim, "Human Tracking and Following using Machine Vision on a Mobile Service Robot," *2022 IEEE 10th Conf. Syst. Process Control. ICSPC 2022 - Proc.*, no. December, pp. 274–279, 2022, doi: 10.1109/ICSPC55597.2022.10001803.

[36] N. Van Toan, M. Do Hoang, P. B. Khoi, and S. Y. Yi, "The human-following strategy for mobile robots in mixed environments," *Rob. Auton. Syst.*, vol. 160, p. 104317, 2023, doi: 10.1016/j.robot.2022.104317.

[37] X. Zhang et al., "Image Recognition of Supermarket Shopping Robot Based on CNN," in *2020 IEEE International Conference on Artificial Intelligence and Computer Applications (ICAICA)*, pp. 1363–1368, 2020. doi: 10.1109/ICAICA50127.2020.9181936.

[38] Y. Li, Y. Chang, T. Yang, and Y. Li, "Understanding consumer responses to robot roles in human-robot service collaboration," *J. Retail. Consum. Serv.*, vol. 87, pp. 1–10, 2025, doi: 10.1016/j.jretconser.2025.104407.

[39] K. Zhu and T. Zhang, "Deep Reinforcement Learning Based Mobile Robot Navigation: A Review," *Tsinghua Sci. Technol.*, vol. 26, no. 5, pp. 674–691, 2021, doi: 10.26599/TST.2021.9010012.

[40] A. Candra, M. A. Budiman, and K. Hartanto, "Dijkstra's and A-Star in Finding the Shortest Path: A Tutorial," in *2020 International Conference on Data Science, Artificial Intelligence, and Business Analytics (DATABIA)*, pp. 28–32, 2020. doi: 10.1109/DATABIA50434.2020.9190342.

[41] D. Rachmawati and L. Gustin, "Analysis of Dijkstra's Algorithm and A∗ Algorithm in Shortest Path Problem," *J. Phys. Conf. Ser.*, vol. 1566, no. 1, pp. 1–7, 2020, doi: 10.1088/1742-6596/1566/1/012061.

[42] G. Tang, C. Tang, C. Claramunt, X. Hu, and P. Zhou, "Geometric A-Star Algorithm: An Improved A-Star Algorithm for AGV Path Planning in a Port Environment," *IEEE Access*, vol. 9, pp. 59196–59210, 2021, doi: 10.1109/ACCESS.2021.3070054.

[43] Y. Li, Z. Wang, and S. Zhang, "Path Planning of Robots Based on an Improved A-star Algorithm," *2022 IEEE 5th Adv. Inf. Manag. Commun. Electron. Autom. Control Conf.*, vol. 5, pp. 826–831, 2022, doi: 10.1109/IMCEC55388.2022.10019799.

[44] L. Zhang and Y. Li, "Mobile Robot Path Planning Algorithm Based on Improved A Star," in *Journal of Physics: Conference Series*, 2021. doi: 10.1088/1742-6596/1848/1/012013.

[45] S. Louda, N. Karkar, F. Seghir, and S. Refoufi, "Mobile Robot Path Planning Based on A-Star Algorithm and Artificial Potential Field Method for Autonomous Navigation," *2024 12th Int. Conf. Syst. Control. ICSC 2024*, pp. 441–446, 2024, doi: 10.1109/ICSC63929.2024.10928867.

[46] Y. Li et al., "A Review of Simultaneous Localization and Mapping Algorithms Based on Lidar," *World Electr. Veh. J.*, vol. 16, no. 2, pp. 1–29, 2025, doi: 10.3390/wevj16020056.

[47] J. Hernas and N. PiÃşrkowska, "Comparison of SLAM algorithms for autonomous navigation systems in ROS 2 environment: analysis of mapping accuracy, position estimation and resource consumption," *Authorea*, 2025, doi: 10.22541/au.175199254.49549720/v1.

[48] S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the Dynamic World," *J. Open Source Softw.*, vol. 6, no. 61, pp. 1–7, 2021, doi: 10.21105/joss.02783.

[49] Z. Zhang and G.-H. Yang, "Dynamic obstacle avoidance for car-like mobile robots based on neurodynamic optimization with control barrier functions," *Neurocomputing*, vol. 654, pp. 1–9, 2025, doi: 10.1016/j.neucom.2025.131252.

[50] C. Su, H. Wang, and C. Ko, "Development of an Autonomous Robot Replenishment System for Convenience Stores," 2023.

[51] Y. Zheng, J. Li, J. Deng, Y. Wang, J. Zhao, and J. Hou, "Research on Local Obstacle Avoidance of Power Inspection Robot Based on Improved TEB-DWA Algorithm," in *2025 5th International Conference on Electronic Technology, Communication and Information (ICETCI)*, IEEE, pp. 737–744, 2025. doi: 10.1109/icetci64844.2025.11084027.

[52] H. Yuan, H. Li, Y. Zhang, S. Du, L. Yu, and X. Wang, "Comparison and Improvement of Local Planners on ROS for Narrow Passages," *2022 Int. Conf. High Perform. Big Data Intell. Syst.*, pp. 125–130, 2022, doi: 10.1109/HDIS56859.2022.9991270.

[53] K. Gong, Z. Xu, and X. Zhang, "Bounded-DWA: An Efficient Local Planner for Ackermann-driven Vehicles on Sandy Terrain," in *2023 IEEE International Conference on Real-Time Computing and Robotics (RCAR)*, IEEE, pp. 632–637, 2023. doi: 10.1109/RCAR58764.2023.10249383.

[54] W. Huang and J. Yuan, "Improvements based on Adaptive Monte Carlo Localization," *2024 5th Int. Conf. Inf. Sci. Parallel Distrib. Syst.*, pp. 103–106, 2024, doi: 10.1109/ISPDS62779.2024.10667543.

[55] D. Ran, C. Huang, Y. Zhang, and W. Ren, "Fast relocation method for indoor mobile robots based on ORB feature matching," *2024 5th Int. Semin. Artif. Intell. Netw. Inf. Technol.*, pp. 64–70, 2024, doi: 10.1109/AINIT61980.2024.10581417.

[56] M.-A. Chung and C.-W. Lin, "An Improved Localization of Mobile Robotic System Based on AMCL Algorithm," *IEEE Sens. J.*, vol. 22, no. 1, pp. 900–908, 2022, doi: 10.1109/JSEN.2021.3126605.

[57] J. Xu, G. Liu, J. Liu, Z. Lv, and S. Gao, "EKF-Based Positioning Study of a Mobile Robot with McNamee Wheels," *J. Phys. Conf. Ser.*, vol. 2281, no. 1, pp. 1–9, 2022, doi: 10.1088/1742-6596/2281/1/012008.

[58] X. Wu, S. Yin, T. Chen, F. Wu, and Z. Huang, "Optimizing Cartographer for Indoor Mapping and Analysis," in *2025 6th International Conference on Computer Engineering and Application (ICCEA)*, pp. 1139–1143, 2025. doi: 10.1109/ICCEA65460.2025.11103219.

[59] O. O. Martins, A. A. Adekunle, O. M. Olaniyan, and B. O. Bolaji, "An Improved multi-objective a-star algorithm for path planning in a large workspace: Design, Implementation, and Evaluation," *Sci. African*, vol. 15, p. e01068, 2022, doi: 10.1016/j.sciaf.2021.e01068.

[60] R. Ni, J. Liu, H. Li, Z. Cao, X. Wang, and Y. Liu, "Research on Robot Path Planning Based on Fusion Algorithm of Optimized A∗ and DWA," in *2024 9th International Conference on Intelligent Computing and Signal Processing (ICSP)*, IEEE, pp. 1436–1439, 2024. doi: 10.1109/ICSP62122.2024.10743661.

[61] M. R. Imamoglu, E. Sumer, and H. Temeltas, "A Comparison of Local Planner Algorithms for a ROS-based Omnidirectional Mobile Robot," in *2023 8th International Conference on Robotics and Automation Engineering (ICRAE)*, IEEE, pp. 26–30, 2023. doi: 10.1109/ICRAE59816.2023.10458649.

[62] J. Palacín, E. Rubies, and E. Clotet, "Systematic Odometry Error Evaluation and Correction in a Human‐Sized Three‐Wheeled Omnidirectional Mobile Robot Using Flower‐Shaped Calibration Trajectories," *Appl. Sci.*, vol. 12, no. 5, pp. 1–23, 2022, doi: 10.3390/app12052606.

[63] G. L. Rezende, F. O. Silva, and L. S. Marques, "Comparison of Wheel Odometry and Visual Odometry for Low-Cost Vehicle Navigation," in *2024 Brazilian Symposium on Robotics (SBR) and 2024 Workshop on Robotics in Education (WRE)*, IEEE, pp. 91–96, 2024. doi: 10.1109/SBR/WRE63066.2024.10838039.

[64] A. de M. Gai, S. Bevilacqua, A. R. Cukla, and D. F. T. Gamarra, "Evaluation on IMU and odometry sensor fusion for a Turtlebot robot using AMCL on ROS framework," in *2023 Latin American Robotics Symposium (LARS), 2023 Brazilian Symposium on Robotics (SBR), and 2023 Workshop of Robotics in Education (WRE)*, IEEE, 2023, pp. 248–253. doi: 10.1109/LARS/SBR/WRE59448.2023.10332977.

[65] A. Barzegar, O. Doukhi, and D.-J. Lee, "Design and Implementation of an Autonomous Electric Vehicle for Self-Driving Control under GNSS-Denied Environments," *Appl. Scienses*, vol. 11, no. 8, pp. 1–23, 2021, doi: 10.3390/app11083688.

[66] I. Ullah, D. Adhikari, H. Khan, M. S. Anwar, S. Ahmad, and X. Bai, "Mobile robot localization: Current challenges and future prospective," *Comput. Sci. Rev.*, vol. 53, pp. 1–34, 2024, doi: 10.1016/j.cosrev.2024.100651.

[67] S. P. Chen, C. Y. Peng, G. S. Huang, C. C. Lai, C. C. Chen, and M. H. Yen, "Comparison of 2D and 3D LiDARs Trajectories and AMCL Positioning in ROS-Based move_base Navigation," in *2023 IEEE International Conference on Omni-Layer Intelligent Systems (COINS)*, IEEE, pp. 1–6, 2023. doi: 10.1109/COINS57856.2023.10189271.

[68] C. Jiang, C. Wang, and M. Wang, "Research on path planning for mobile robots based on improved A-star algorithm," in *2023 IEEE 7th Information Technology and Mechatronics Engineering Conference (ITOEC)*, IEEE, pp. 723–727, 2023. doi: 10.1109/ITOEC57671.2023.10292045.

[69] Y. Song and P. Ma, "Research on Mobile Robot Path Planning Based on Improved A-star Algorithm," in *2021 International Conference on Electronic Information Engineering and Computer Science (EIECS)*, IEEE, pp. 683–687, 2021. doi: 10.1109/EIECS53707.2021.9588002.

[70] Z. He, C. Liu, X. Chu, R. R. Negenborn, and Q. Wu, "Dynamic Anti-Collision A-Star Algorithm for Multi-Ship Encounter Situations," *Appl. Ocean Res.*, vol. 118, no. 1, pp. 2–26, 2022, doi: 10.1016/j.apor.2021.102995.

[71] H. Kuang, Y. Li, Y. Zhang, and Y. Feng, "Improved A-star Algorithm based on Topological Maps for Indoor Mobile Robot Path Planning," in *IEEE 6th Information Technology and Mechatronics Engineering Conference (ITOEC)*, IEEE, pp. 1236–1240, 2022. doi: 10.1109/ITOEC53115.2022.9734323.

[72] S. Guo, X. Pan, and Z. Liu, "AGV Path Planning Algorithm Based on Fusion of Improved A∗ and DWA," in *Proceedings of the 43rd Chinese Control Conference (CCC)*, Technical Committee on Control Theory, Chinese Association of Automation, pp. 1782–1787, 2024. doi: 10.23919/CCC63176.2024.10662786.

[73] Z. Lin and R. Taguchi, "Faster Implementation of The Dynamic Window Approach Based on Non-Discrete Path Representation," *Math. Artic.*, vol. 11, no. 21, pp. 1–20, 2023, doi: doi.org/10.3390/math11214424.

[74] S. M. Sakti, A. D. Laksito, B. W. Sari, and D. Prabowo, "Music Recommendation System Using Content-based Filtering Method with Euclidean Distance Algorithm," in *2022 6th International Conference on Information Technology, Information Systems and Electrical Engineering (ICITISEE)*, IEEE, pp. 385–390, 2022. doi: 10.1109/ICITISEE57756.2022.10057753.

[75] R. Mussabayev, "Optimizing Euclidean Distance Computation," *Mathematics*, vol. 12, no. 23, pp. 1–36, 2024, doi: 10.3390/math12233787.

[76] X. Li, X. Hu, Z. Wang, and Z. Du, "Path planning based on combination of improved A-STAR Algorithm and DWA algorithm," *Proc. - 2020 2nd Int. Conf. Artif. Intell. Adv. Manuf. AIAM 2020*, no. 1, pp. 99–103, 2020, doi: 10.1109/AIAM50918.2020.00025.

[77] P. Li, L. Hao, Y. Zhao, and J. Lu, "Robot obstacle avoidance optimization by A* and DWA fusion algorithm," *PLoS One*, vol. 19, no. 4 April, pp. 1–21, 2024, doi: 10.1371/journal.pone.0302026.

[78] D. Wang, J. Zhang, and W. Wang, "A Conflict Resolution Method for Multi-Robot Path Planning Based on an Improved Variable-Weight Dynamic Window Approach," *2025 17th Int. Conf. Adv. Comput. Intell.*, no. Icaci, pp. 107–112, 2025, doi: 10.1109/icaci65340.2025.11096346.

[79] B. Hahn, "Enhancing Obstacle Avoidance in Dynamic Window Approach via Dynamic Obstacle Behavior Prediction," *Actuators*, vol. 14, no. 5, 2025, doi: 10.3390/act14050207.

[80] C. Xu, J. Park, and J. C. Lee, "The effect of shopping channel (online vs offline) on consumer decision process and firm's marketing strategy," *Internet Res.*, vol. 32, no. 3, pp. 971–987, 2022, doi:
