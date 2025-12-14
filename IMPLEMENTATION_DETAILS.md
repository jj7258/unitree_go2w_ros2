# Go2W Implementation Details

This document provides a comprehensive log of the engineering work, code modifications, and design decisions involved in creating the `go2w_ws` workspace for the Unitree Go2W (Wheeled) robot.

---

## 1. Description Package Transformation (`go2w_description`)

**Goal**: Port the robot description from ROS1 to ROS2 and verify 16-motor support.

### 1.1 From Catkin to Ament
The original `go2w_description` was obtained from [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros). It used the legacy `catkin` build system.

**Changes made to `CMakeLists.txt`**:
```cmake
# Old (ROS1)
# cmake_minimum_required(VERSION 2.8.3)
# find_package(catkin REQUIRED)
# catkin_package()

# New (ROS2)
cmake_minimum_required(VERSION 3.8)
project(go2w_description)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf dae config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
```

**Changes made to `package.xml`**:
Changed build tool from `<buildtool_depend>catkin</buildtool_depend>` to `<buildtool_depend>ament_cmake</buildtool_depend>`.

### 1.2 Launch Architecture
ROS1 uses `.launch` XML files. ROS2 uses Python launch scripts. We created `launch/robot.launch.py`:

```python
# loads xacro and publishes robot state
robot_description_content = Command(
    [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
     PathJoinSubstitution([FindPackageShare('go2w_description'), 'urdf', description_file])]
)
node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description_content}]
)
```

## 2. Driver Adaptation (`go2w_driver`)

**Goal**: Interface with the Go2W's 16-motor LowState system.

### 2.1 Namespace & Identity
Forked from `go2_driver`.
- **Global Rename**: `go2_driver` -> `go2w_driver`.
- **Plugin Registration**: In `CMakeLists.txt`, ensured the component is registered correctly:
  ```cmake
  rclcpp_components_register_nodes(${PROJECT_NAME} "go2w_driver::Go2Driver")
  ```

### 2.2 The 16-Motor Challenge
The standard Go2 has 12 motors (3 per leg). The Go2W has 4 additional wheel motors. We verified via the Unitree SDK that these occupy indices 12-15 in the `LowState` array.

**Code Modification (`go2w_driver.cpp`)**:
We updated the `publish_joint_states` function to map these extra indices to the wheel joints defined in the URDF.

```cpp
void Go2Driver::publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg)
{
  // ...
  joint_state.name = {
    // Standard Legs
    "FL_hip_joint", ..., "RR_foot_joint",
    // NEW: Wheels
    "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"
  };

  joint_state.position = {
    // Standard Mappings (0-11)
    msg->motor_state[3].q, ... ,
    // NEW: Wheel Mappings (12-15)
    msg->motor_state[12].q, // FL Wheel
    msg->motor_state[13].q, // FR Wheel
    msg->motor_state[14].q, // RL Wheel
    msg->motor_state[15].q  // RR Wheel
  };
  // ...
}
```

## 3. Mock Simulation (`go2w_mock`)

**Goal**: Verification without hardware.

Since the `go2w_driver` connects to a real robot (or Unitree's binary-only simulation), we needed a way to verify our ROS2 stack (TFs, URDF, Driver logic) independently.

### 3.1 Kinematic Simulation
We implemented a custom Python node `go2w_mock/mock_node.py` that acts as a "Virtual Robot".

- **Joint Handling**: Interpolates leg joints to "Stand", "Sit", or "Lie Down" poses based on `api/sport/request` commands.
- **Wheel Integration**:
    - Unlike legs (position control simulation), wheels are simulated as **velocity integrated**.
    - If `cmd_vel` is received, the mock integrates velocity over time to spin the wheel joints infinitely: `pos += vel * dt`.

### 3.2 Odometry Simulation
To test navigation stacks, we needed the robot to "move" in the world frame.

```python
# go2w_mock/mock_node.py

# Simple Skid-Steer Model
self.robot_theta += self.angular_z * dt
self.robot_x += self.linear_x * math.cos(self.robot_theta) * dt
self.robot_y += self.linear_x * math.sin(self.robot_theta) * dt

# Publish fake pose to satisfy driver
pose_msg = PoseStamped()
pose_msg.header.frame_id = "odom"
self.pose_pub.publish(pose_msg) # Topic: /utlidar/robot_pose
```

## 4. Dependencies & Environment

- **Dependencies**: `unitree_sdk2`, `unitree_ros2` (for message definitions).
- **Environment**: We explicitly use `noenv` in `.bashrc` to avoid Conda conflicts, ensuring the system ROS2 libraries are used.
