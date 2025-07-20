# Digital Twin for Vision-Based Robotic Arm Manipulation

![Robot Arm](icon.png)
This repository showcases a digital twin system for simulating and controlling a robotic arm (Franka Emika Panda) to detect and pick up plastic bottles in a virtual environment. The system integrates Isaac Sim for high-fidelity simulation, YOLOv11 for object detection, SAM (Segment Anything Model) for segmentation, MoveIt 2 for motion planning, and ROS 2 as the communication middleware.

The goal is to build a scalable foundation for vision-based robotic manipulation with a focus on waste sorting and environmental automation.
---

## 🎯Objective

To build a bridge between **perception** and **robotic action** by integrating:

- ✅ **YOLO11s object detection** (vision)
- ✅ **Isaac Sim robot simulation** (digital twin)
- ✅ **ROS 2** for communication
- ✅ **Predefined motion sequences** using joint control

The robot performs **autonomous pick-up actions** only when the YOLO model identifies an object as `"bottle"`.

---

## 🛠️ Tech Stack Overview

| **Component**          | **Role**                                                                 |
|------------------------|--------------------------------------------------------------------------|
| **Isaac Sim**          | Simulates the robot, physics-based environment, camera, and sensors      |
| **Franka Emika Panda** | Performs pick-and-place operations using joint-level commands            |
| **YOLOv11**            | Detects plastic bottles from the simulated camera feed                   |
| **SAM (Segment Anything)** | Segments objects to improve visual understanding and manipulation  |
| **MoveIt 2**           | Plans and executes robotic arm trajectories for picking and placing      |
| **ROS 2**              | Acts as middleware to coordinate detection, planning, and control        |
| **cv_bridge**          | Converts ROS 2 image messages to OpenCV format for YOLO/SAM processing   |
| **Python**             | Core scripting language for integrating perception, planning, and control|
| **Rviz2 / Isaac UI**   | Visualizes robot states, environment, and sensor data                     |

---
## 🧱🧊 Simulation Scene & Layout
The scene contains a quad (plane) with 5 plastic bottles and cubes arranged on it.

These objects are static (i.e., not moving) and spawned at predefined coordinates inside Isaac Sim.

The robot arm (Franka Emika Panda) is placed near the quad with an RGB camera mounted on its wrist for dynamic perception.

## 🗺️ Bottle Positions & Predefined Joints
The positions of the 5 bottles are known in advance.

Instead of computing inverse kinematics in real-time, the project uses a predefined joint configuration for each bottle.

Each configuration consists of specific joint angles that allow the arm to reach a particular bottle's location.

These joint values are stored in a JSON file where:

Each key represents a bottle ID or label (e.g., "bottle_1")

Each value contains the list of 7 joint angles needed to move to that bottle.


## Camera Image Flow (ROS + YOLO) — 🔬 In Detail

Step 1: Capture Image from Isaac Sim
The RGB camera mounted on the robotic arm captures real-time images of the environment inside Isaac Sim.

Step 2: Stream via ROS
These images are streamed out of the simulator using ROS as image messages, making them accessible to external processes.

Step 3: Image Processing for YOLO
The streamed images are converted into a format compatible with the YOLO11s model (typically NumPy arrays or OpenCV images).

Step 4: Object Detection (YOLO)
YOLO11s processes each frame and detects plastic bottles present in the scene. Each detection outputs a label, confidence, and bounding box.

Step 5: Detection Signal via ROS
If a "bottle" is detected, a detection signal (label info) is published to a ROS topic for downstream motion control.

Robot Motion Flow —  Predefined Joint Execution
Step 1: Wait for Detection
A ROS node keeps listening for incoming detection labels (like "bottle") from the YOLO pipeline.

Step 2: Lookup Bottle Position
Each bottle's position is pre-defined in the simulation. When a detection occurs, the node finds the joint angles associated with that bottle’s location.

Step 3: Move Using Predefined Joints
The robot is instructed to move by setting its joints to these predefined angles using Isaac Sim's articulation API.

Step 4: Grasp the Bottle
Once the arm reaches the target position, the gripper closes around the bottle to complete the pick-up.

Step 5: Reset or Wait
The robot either returns to a home position or pauses, waiting for the next detection to repeat the cycle.
```bash
/simulated_camera/image_raw
