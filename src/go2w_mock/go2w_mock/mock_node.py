#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import math
import time

from unitree_go.msg import LowState, MotorState, IMUState
from unitree_api.msg import Request
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

# Define basic joint poses
# Layout: [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf, FL_wheel, FR_wheel, RL_wheel, RR_wheel]
# Note: LowState message uses a fixed array of 20 motors.
# go2_driver mapping seems to be: 
#   3,4,5 -> FL (hip, thigh, calf)
#   0,1,2 -> FR 
#   9,10,11 -> RL
#   6,7,8 -> RR
#   12,13,14,15 -> Wheels (FL, FR, RL, RR) -- Verified from SDK analysis 

# Standard Stand Pose (approximate)
STAND_JOINTS = {
    0: -0.1, 1: 0.8, 2: -1.5,
    3: 0.1, 4: 0.8, 5: -1.5,
    6: -0.1, 7: 0.8, 8: -1.5,
    9: 0.1, 10: 0.8, 11: -1.5
}
# Wheels (12-15) initially 0

# Sit Pose
SIT_JOINTS = {
    0: -0.1, 1: 1.1, 2: -2.0,   # FR
    3: 0.1, 4: 1.1, 5: -2.0,    # FL
    6: -0.1, 7: 1.1, 8: -2.0,   # RR
    9: 0.1, 10: 1.1, 11: -2.0   # RL
}

# Lie Down Pose
LIE_DOWN_JOINTS = {
    0: -0.5, 1: 1.5, 2: -2.5,
    3: 0.5, 4: 1.5, 5: -2.5,
    6: -0.5, 7: 1.5, 8: -2.5,
    9: 0.5, 10: 1.5, 11: -2.5
}

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        self.low_state_pub = self.create_publisher(LowState, 'lowstate', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/utlidar/robot_pose', 10)
        self.api_sub = self.create_subscription(Request, 'api/sport/request', self.api_callback, 10)
        
        self.timer = self.create_timer(0.02, self.publish_state) # 50Hz
        
        self.current_joints = {}
        for i in range(20):
            self.current_joints[i] = 0.0
            
        # Initialize to Stand
        for k, v in STAND_JOINTS.items():
            self.current_joints[k] = v
            
        self.target_joints = self.current_joints.copy()
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0] # FL, FR, RL, RR
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.gait_enabled = False
        self.gait_type = 0 # 0: Idle, 1: Trot
        self.start_time = time.time()
        self.last_time = time.time()
        
        self.get_logger().info("Go2W Mock Robot Started")

    def api_callback(self, msg):
        try:
            req_id = msg.header.identity.api_id
            if msg.parameter:
                try:
                    data = json.loads(msg.parameter)
                except json.JSONDecodeError:
                    data = {}
            else:
                data = {}
            
            self.get_logger().info(f"Received API Request: ID={req_id}, Data={data}")
            
            # IDs based on go2_driver/include/go2_driver/go2_api_id.hpp
            # StandUp = 1004
            # StandDown = 1005 (Lie down?)
            # Sit = 1009
            # Move = 1008
            
            if req_id == 1004: # StandUp
                self.set_target_pose(STAND_JOINTS)
                self.stop_wheels()
            elif req_id == 1009: # Sit
                self.set_target_pose(SIT_JOINTS)
                self.stop_wheels()
            elif req_id == 1005: # StandDown
                self.set_target_pose(LIE_DOWN_JOINTS)
                self.stop_wheels()
            elif req_id == 1008: # Move (cmd_vel)
                x = data.get('x', 0.0)
                y = data.get('y', 0.0) # Not used for diff/skid steer usually
                z = data.get('z', 0.0) # Angular Z
                self.set_wheel_velocity(x, z)
            elif req_id == 1003: # StopMove
                self.stop_wheels()
            elif req_id == 1011: # SwitchGait
                val = data.get('d', 0) # 'd' seems to be the key from user logs? or 'data'? 
                # User log: Data={'data': 1}. Wait, the log says Data={'data': 1}.
                # The cheatsheet said "{d: 1}". The user log says "{'data': 1}". 
                # Let's check the user log again. 1765745858.496519606 ... Data={'data': 1}
                # Ah, my cheatsheet might have been "d" based on proto, but the python repr might be "data". 
                # I'll check both.
                val = data.get('data', data.get('d', 0))
                self.gait_type = val
                self.get_logger().info(f"Switched Gait to {val}")
            elif req_id == 1019: # ContinuousGait
                # User log: Data={'data': True}
                flag = data.get('data', data.get('flag', False))
                self.gait_enabled = bool(flag)
                self.get_logger().info(f"Continuous Gait set to {self.gait_enabled}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to parse API request: {e}")

    def set_target_pose(self, pose_dict):
        for k, v in pose_dict.items():
            self.target_joints[k] = v
            
    def stop_wheels(self):
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0]
        
    def set_wheel_velocity(self, linear_x, angular_z):
        # Simple Skid Steer approximation
        # Left wheels: FL(12), RL(14)
        # Right wheels: FR(13), RR(15)
        # go2_driver map: 12->FL_foot, 13->FR_foot, 14->RL_foot, 15->RR_foot
        
        width = 0.4 # Approx robot width
        left = linear_x - (angular_z * width / 2.0)
        right = linear_x + (angular_z * width / 2.0)
        
        # Mapping: 12=FL(L), 13=FR(R), 14=RL(L), 15=RR(R) (Check URDF/Driver map)
        # Driver map from step 245:
        # msg->motor_state[12] -> FL_foot
        # msg->motor_state[13] -> FR_foot
        # msg->motor_state[14] -> RL_foot
        # msg->motor_state[15] -> RR_foot
        
        # Assuming odd=Right, even=Left is unlikely or likely?
        # Standard Unitree: 
        # 0-2 FR, 3-5 FL, 6-8 RR, 9-11 RL
        # So 12 should be FL_wheel, 13 FR_wheel ?? 
        # URDF: FL_foot is left.
        # Let's assume 12=FL, 13=FR, 14=RL, 15=RR
        
        self.wheel_vel[0] = left  # 12
        self.wheel_vel[1] = right # 13
        self.wheel_vel[2] = left  # 14
        self.wheel_vel[3] = right # 15

    def publish_state(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        msg = LowState()
        
        # Interpolate joints
        alpha = 0.1 # Smoothing factor
        for i in range(20):
            # Logic for wheels (infinite rotation)
            if i >= 12 and i <= 15:
                # Wheels: Integrate velocity simply, do NOT interpolate to target
                vel = self.wheel_vel[i - 12]
                self.current_joints[i] += vel * dt * 10.0 # Scale factor for speed
                msg.motor_state[i].dq = vel # Velocity feedback
            else:
                # Joints: Interpolate to target
                # Add gait animation if enabled
                anim_offset = 0.0
                if self.gait_enabled or (self.gait_type != 0 and (abs(self.linear_x) > 0.01 or abs(self.angular_z) > 0.01)):
                     # Simple sine wave trot
                     # Freq ~ 2Hz
                     t = time.time() - self.start_time
                     freq = 3.0
                     amp_thigh = 0.3
                     amp_calf = 0.3
                     
                     # Diagonals: (FR, RL) vs (FL, RR)
                     # FR(0..2), RL(9..11) -> Phase 0
                     # FL(3..5), RR(6..8) -> Phase PI
                     
                     phase = 0.0
                     if (i >= 3 and i <= 5) or (i >= 6 and i <= 8): # FL or RR
                        phase = math.pi
                        
                     # Only animate Thigh and Calf
                     # indices % 3: 0->Hip, 1->Thigh, 2->Calf
                     # Hip (0, 3, 6, 9) -> small sway?
                     # Thigh (1, 4, 7, 10)
                     # Calf (2, 5, 8, 11)
                     
                     motor_idx_mod = i % 3  # Assuming standard 0-11 mapping for legs
                     if motor_idx_mod == 1: # Thigh
                        anim_offset = math.sin(2*math.pi*freq*t + phase) * amp_thigh
                     elif motor_idx_mod == 2: # Calf
                        anim_offset = math.sin(2*math.pi*freq*t + phase + 1.0) * amp_calf
                        
                current = self.current_joints[i]
                target = self.target_joints[i]
                self.current_joints[i] = current + (target - current) * alpha + (anim_offset * 0.1) # Mix animation with pose
            
            msg.motor_state[i].q = self.current_joints[i]
            
        self.low_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
