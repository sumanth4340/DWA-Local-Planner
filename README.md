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

<img width="1222" height="878" alt="Screenshot 2026-02-05 131608" src="https://github.com/user-attachments/assets/2d2bab9d-fe74-4469-8f5c-bf9c17d7a5d2" />
<img width="1919" height="1079" alt="Screenshot 2026-02-05 130829" src="https://github.com/user-attachments/assets/90546a3c-86f3-4372-b4ed-f88834bcae9e" />
<img width="1455" height="748" alt="Screenshot 2026-02-05 131637" src="https://github.com/user-attachments/assets/6e035885-132f-4022-8e01-d0833f9cf767" />
<img width="1395" height="722" alt="Screenshot 2026-02-05 131622" src="https://github.com/user-attachments/assets/6a041f81-a695-443e-8677-257000dc3e4a" />

### TURTLEBOT

https://github.com/user-attachments/assets/7fd4d36a-a94c-409e-adb4-51a786a54b55







