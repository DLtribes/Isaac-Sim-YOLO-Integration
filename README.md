# Isaac-Sim-YOLO-Integration


This repository demonstrates **digital twin system** where the **robotic arm (Franka Emika Panda)** detects and picks up **plastic bottles** in a simulated Isaac Sim environment using a **YOLOv11 detection model** and **ROS 2** middleware.

---

## 🎯Objective

To build a bridge between **perception** and **robotic action** by integrating:

- ✅ **YOLO11-OBB object detection** (vision)
- ✅ **Isaac Sim robot simulation** (digital twin)
- ✅ **ROS 2** for communication
- ✅ **Predefined motion sequences** using joint control

The robot performs **autonomous pick-up actions** only when the YOLO model identifies an object as `"bottle"`.

---

## 🧠 System Overview

| Component            | Role                                                                 |
|---------------------|----------------------------------------------------------------------|
| **Isaac Sim**        | Simulates the robot, scene, and camera                               |
| **Franka Robot**     | Executes pick-and-place based on predefined joint targets            |
| **YOLOv8**           | Detects objects in simulated camera feed                             |
| **ROS 2**            | Handles messaging between detection and motion systems               |
| **cv_bridge**        | Converts ROS images into OpenCV format for model input               |

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
The streamed images are converted into a format compatible with the YOLOv8 model (typically NumPy arrays or OpenCV images).

Step 4: Object Detection (YOLOv8)
YOLOv8 processes each frame and detects plastic bottles present in the scene. Each detection outputs a label, confidence, and bounding box.

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
