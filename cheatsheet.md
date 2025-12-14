# Go2W ROS2 Service Call Cheatsheet

Here is the complete list of available service calls for the Go2W robot.

## 1. Robot Modes (`/mode`)
Change the high-level behavior of the robot.

**Common Modes:**
```bash
# Stand Up (prepare for walking)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stand_up'}"

# Stand Down (lie flat)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stand_down'}"

# Sit
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'sit'}"

# Balance Stand (maintain balance)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'balance_stand'}"

# Damp (relax joints, be careful!)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'damp'}"

# Stop/Recover
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stop_move'}"
```

**Fun/Special Modes:**
```bash
# Hello (Wave)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'hello'}"

# Stretch
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stretch'}"

# Dance 1
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'dance1'}"

# Dance 2
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'dance2'}"

# Handstand (Front Jump/Pounce - verify safety first!)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'front_jump'}"
```

## 2. Locomotion Control

**Switch Gait (`/switch_gait`)**
Change walking style (0: Idle, 1: Trot, 2: Trot Running, 3: Climb Stairs, 4: Crawl).
```bash
# Switch to Trot
ros2 service call /switch_gait go2_interfaces/srv/SwitchGait "{d: 1}"
```

**Speed Level (`/speed_level`)**
Adjust speed range (-1: Low, 0: Normal, 1: High).
```bash
# Set High Speed
ros2 service call /speed_level go2_interfaces/srv/SpeedLevel "{level: 1}"
```

**Continuous Gait (`/continuous_gait`)**
Enable/Disable continuous movement (1: On, 0: Off).
```bash
ros2 service call /continuous_gait go2_interfaces/srv/ContinuousGait "{flag: 1}"
```

## 3. Body Adjustments

**Body Height (`/body_height`)**
Adjust the robot's standing height (Range: approx -0.18 to 0.03 relative to default).
```bash
# Lower body
ros2 service call /body_height go2_interfaces/srv/BodyHeight "{height: -0.1}"
```

**Foot Raise Height (`/foot_raise_height`)**
Adjust how high feet lift during walking (Range: 0.0 to 0.1m).
```bash
# High stepping
ros2 service call /foot_raise_height go2_interfaces/srv/FootRaiseHeight "{height: 0.08}"
```

**Euler/Attitude (`/euler`)**
Control the body orientation (Roll, Pitch, Yaw).
```bash
# Lean forward (Pitch)
ros2 service call /euler go2_interfaces/srv/Euler "{roll: 0.0, pitch: 0.3, yaw: 0.0}"
```

**Pose (`/pose`)**
Toggle pose mode (Usually for testing specific poses, flag: 1/0).
```bash
ros2 service call /pose go2_interfaces/srv/Pose "{flag: 1}"
```

## 4. Joystick Control
**Switch Joystick (`/switch_joystick`)**
Enable/Disable internal joystick response (1: On, 0: Off).
```bash
ros2 service call /switch_joystick go2_interfaces/srv/SwitchJoystick "{flag: 0}"
```
