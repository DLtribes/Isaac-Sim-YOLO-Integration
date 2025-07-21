# Digital Twin for Vision-Based Robotic Arm Manipulation

![Robot Arm](icon.png)

Reflect is a robotics-driven digital twin system designed for smart waste management. The project creates a real-time mirror of a physical robotic arm using Isaac Sim, ROS 2, and MoveIt 2, enabling precise simulation and control.
It leverages YOLOv11 for object detection and semantic understanding of waste items, with ongoing integration of pose estimation and reinforcement learning to improve pick-and-place accuracy and decision-making. Reflect aims to reduce physical testing overhead, support sustainability, and bridge human-AI collaboration—symbolized by its logo: a robotic hand reaching out to a human hand, echoing the project’s core theme of synchronization and reflection.

---

## Objective

To build a bridge between **perception** and **robotic action** by integrating:

- **YOLO11s object detection** (vision)
- **Isaac Sim robot simulation** (digital twin)
- **ROS 2** for communication
- **Predefined motion sequences** using joint control

The robot performs **autonomous pick-up actions** only when the YOLO model identifies an object as `"bottle"`.

---

## Tech Stack Overview

| **Component**          | **Role**                                                                 |
|------------------------|--------------------------------------------------------------------------|
| **Isaac Sim**          | Simulates the robot, physics-based environment, camera, and sensors      |
| **YOLOv11**            | Detects plastic bottles from the simulated camera feed                   |
| **SAM (Segment Anything)** | Segments objects to improve visual understanding and manipulation  |
| **MoveIt 2**           | Plans and executes robotic arm trajectories for picking and placing      |
| **ROS 2**              | Acts as middleware to coordinate detection, planning, and control        |
| **Rviz2 / Isaac UI**   | Visualizes robot states, environment, and sensor data                     |

---
## SimTest

### 3D Bottle Detection Test

<div align="center">
  <img src="sim_3d.gif" alt="Robot Arm" width="600"/>
</div>


We conducted a focused test using a **single bottle image**, placed in a controlled environment. A **finetuned YOLOv11 model** was used to detect the bottle from the simulated camera feed in Isaac Sim.

This test helped validate:
- The model’s ability to recognize a single instance in isolation  
- Integration with the robot’s response system (MoveIt and joint commands)

### Bottles on Quad: Detection & Motion Refinement


<div align="center">
  <img src="sim_quad.gif" alt="Robot Arm" width="600"/>
</div>


We designed a scene using a **quad surface** where several **cubes** were placed. Each cube displayed an **image of five bottles**, all in different orientations to simulate cluttered and varied real-world conditions. The **Franka Emika Panda** robot, running inside **Isaac Sim**, was used to detect these bottles using a **finetuned YOLOv11 model**. The simulated camera feed provided by Isaac was used as the input for detection, allowing the robot to identify objects directly from the textured cube faces. To enhance the motion quality, a **smoothing function** was implemented. This specifically addressed the issue of **inertial oscillations**—a common artifact in simulated robots when performing rapid joint movements. The function ensures:
- Reduced jitter and swing during arm transitions  
- Smoother, more natural movements  
- A more stable digital twin experience during pick-and-place operations

```bash
/simulated_camera/image_raw
