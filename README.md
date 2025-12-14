# Unitree Go2W ROS2 Workspace

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2W-green.svg)
![Status](https://img.shields.io/badge/Status-Verified-success.svg)

![Unitree Go2W](media/GO2W.gif)

This workspace contains the complete ROS2 integration for the **Unitree Go2W (Wheeled)** robot. Unlike the standard Go2, the Go2W features 16 active degrees of freedom (12 leg motors + 4 wheel motors). This workspace provides the drivers, description, and simulation tools necessary to control all 16 motors. Be sure to check [cheatsheet.md](cheatsheet.md) for a quick reference on common commands and service calls.

## 📂 Repository Structure

```
go2w_ws/
├── src/
│   ├── go2w_bringup/       # Launch files and orchestration
│   ├── go2w_description/   # URDF, Xacro, Meshes (Converted to ROS2)
│   ├── go2w_driver/        # C++ Driver for 16-motor control
│   └── go2w_mock/          # Python-based Kinematic Simulation
├── cheatsheet.md           # Quick reference for Service Calls
├── IMPLEMENTATION_DETAILS.md # Deep dive into code interactions
└── README.md
```

## 🚀 Getting Started

### Prerequisites

1.  **OS**: Ubuntu 22.04 LTS
2.  **ROS2**: Humble Hawksbill
3.  **Dependencies**:
    *   `unitree_sdk2`
    *   `unitree_ros2` (for message definitions like `unitree_go`)

### Installation

1.  **Clone the workspace**:
    ```bash
    git clone <your-repo-url> go2w_ws
    cd go2w_ws
    ```

2.  **Build**:
    ```bash
    colcon build
    source install/setup.bash
    ```

---

## 🎮 Usage Guide

### 1. 🖥️ Mock Simulation (No Robot Required)
Perfect for testing your code, visualization, and navigation logic without hardware.

**Launch**:
```bash
ros2 launch go2w_bringup go2w.launch.py rviz:=True sim:=True
```

**What to expect**:
- **RViz** will open showing the Go2W model.
- **TF Tree**: `odom -> base_link -> ... -> FL_foot_motor` (wheels).
- **Control**: You can send commands to `/cmd_vel`.

### 2. 🤖 Real Robot Deployment
Deploying to the actual Go2W robot or an onboard PC.

**Prerequisite**: Ensure the Unitree LiDAR SDK service is running on the robot (provides `/utlidar/robot_pose`).

**Launch**:
```bash
ros2 launch go2w_bringup go2w.launch.py
```

---

## 🕹️ Control Interfaces

### Velocity Control (Wheels & Legs)
The driver intelligently forwards velocity commands to the active locomotion controller (Legs or Wheels).

**Topic**: `/cmd_vel` (`geometry_msgs/Twist`)
```bash
# Move Forward 0.5 m/s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Spin Left 0.5 rad/s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

### High-Level Modes
Use ROS2 Services to switch robot behaviors. See [cheatsheet.md](cheatsheet.md) for the full list.

```bash
# Stand Up
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stand_up'}"

# Sit Down
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'sit'}"
```

### Gait Control
```bash
# Switch to Trot (while moving)
ros2 service call /switch_gait go2_interfaces/srv/SwitchGait "{d: 1}"
```

---

## 🛠️ Implementation Details
For a technical breakdown of how we ported the description from ROS1, how we mapped the 16 motors in C++, and how the mock simulation works, read the **[Implementation Details](IMPLEMENTATION_DETAILS.md)**.

## 📚 References & Acknowledgements

*   **[Unitree Robotics](https://www.unitree.com/)**: For the hardware, SDK2, and base ROS2 packages.
*   **[Intelligent Robotics Lab (URJC)](https://github.com/IntelligentRoboticsLabs)**: For the original `go2_driver` implementation which served as the foundation for this work.
*   **[unitree_ros](https://github.com/unitreerobotics/unitree_ros)**: Source for the mesh files and URDF structure.
