# DWA Local Planner

A **custom implementation of the Dynamic Window Approach (DWA)** for autonomous navigation of a TurtleBot using **ROS 2 Humble**.  
This project computes safe and smooth velocity commands (`cmd_vel`) in real time using sensor feedback, without relying on Nav2’s built-in planners.

---

## 🔹 Overview
The Dynamic Window Approach (DWA) is a local planning algorithm that samples possible velocity commands, predicts trajectories, evaluates them using a cost function, and selects the optimal command for navigation while avoiding obstacles.

This project demonstrates a **from-scratch DWA implementation** with visualization and simulation testing.

---

## ✨ Key Features
- Custom DWA local planner implementation
- Velocity sampling within robot dynamic constraints
- Trajectory prediction using motion model
- Cost-based trajectory evaluation:
  - Goal distance
  - Obstacle avoidance
  - Motion smoothness
- Real-time `cmd_vel` generation
- RViz trajectory and marker visualization
- Tested on TurtleBot simulation

---

## Algorithm Workflow
1. Read robot state from `/odom`
2. Detect obstacles using `/scan`
3. Sample linear and angular velocities
4. Predict trajectories for each velocity sample
5. Compute cost for each trajectory
6. Select the best trajectory
7. Publish optimized `cmd_vel`

---

##  Implementation Steps

### 1. ROS 2 Workspace Setup
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Package Creation
````
cd ~/ros2_ws/src
ros2 pkg create dwa_planner --build-type ament_python \
--dependencies rclpy geometry_msgs nav_msgs sensor_msgs visualization_msgs
````

### 3. RUN 

````
source ~/ros2_ws/install/setup.bash
ros2 run dwa_planner dwa.py
````
<img width="1475" height="733" alt="Screenshot 2026-02-03 134900" src="https://github.com/user-attachments/assets/af1d4180-50e0-49a7-824f-26bff5810d94" />
<img width="1841" height="961" alt="Screenshot 2026-02-03 131815" src="https://github.com/user-attachments/assets/fe7fdab2-3b7d-445f-900a-9d8e6f0f5ea9" />
<img width="1551" height="920" alt="Screenshot 2026-02-03 135054" src="https://github.com/user-attachments/assets/ed961c19-47bc-4d3c-84a8-1befa93914af" />
<img width="1457" height="725" alt="Screenshot 2026-02-03 135015" src="https://github.com/user-attachments/assets/0ce9a068-a8ed-4925-9517-75cf76d8885f" />


