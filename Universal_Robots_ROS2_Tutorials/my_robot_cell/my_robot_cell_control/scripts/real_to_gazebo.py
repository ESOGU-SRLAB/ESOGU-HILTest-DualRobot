#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class RealToSimJointSync(Node):
    def __init__(self):
        super().__init__('real_to_sim_joint_sync')
        
        # Subscribe to the real robot's joint states
        self.real_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.real_joint_callback,
            10
        )
        
        # Publish joint trajectory commands to the simulation controller
        self.sim_joint_pub = self.create_publisher(
            JointTrajectory,
            '/sim/sim_scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscribe to the simulation's joint states for debugging
        self.sim_joint_sub = self.create_subscription(
            JointState,
            '/sim/joint_states',
            self.sim_joint_callback,
            10
        )
        
        # Joint mapping: real_joint_name -> sim_joint_name
        # This mapping is crucial because the names are different.
        # Use the names from your 'ros2 topic echo' outputs.
        self.joint_mapping = {
            'ur10e_shoulder_pan_joint': 'sim_ur10e_shoulder_pan_joint',
            'ur10e_shoulder_lift_joint': 'sim_ur10e_shoulder_lift_joint',
            'ur10e_elbow_joint': 'sim_ur10e_elbow_joint',
            'ur10e_wrist_1_joint': 'sim_ur10e_wrist_1_joint',
            'ur10e_wrist_2_joint': 'sim_ur10e_wrist_2_joint',
            'ur10e_wrist_3_joint': 'sim_ur10e_wrist_3_joint'
        }
        
        # Simulation joint names in the order expected by the controller
        # This list should match the keys of your joint_mapping
        self.sim_joint_names = [
            'sim_ur10e_shoulder_lift_joint',
            'sim_ur10e_elbow_joint',
            'sim_ur10e_wrist_1_joint',
            'sim_ur10e_wrist_2_joint',
            'sim_ur10e_shoulder_pan_joint',
            'sim_ur10e_wrist_3_joint'
        ]
                
        # Reverse mapping for easier lookup
        # sim_joint_name -> real_joint_name
        self.reverse_mapping = {v: k for k, v in self.joint_mapping.items()}
        
        # Last received real joint positions and velocities
        self.last_real_positions = None
        self.last_real_velocities = None
        
        # Rate limiting for publishing commands
        self.last_command_time = self.get_clock().now()
        self.command_rate_limit = 0.02  # 50 Hz max
        
        # Debug counters
        self.message_count = 0
        self.success_count = 0
        self.error_count = 0
        
        self.get_logger().info('Real to Sim Joint Synchronizer started.')
        self.get_logger().info(f'Joint mapping: {self.joint_mapping}')
        self.get_logger().info(f'Sim joint order: {self.sim_joint_names}')

    def real_joint_callback(self, msg):
        """Processes joint states from the real robot"""
        self.message_count += 1
        
        try:
            self.get_logger().debug(f'--- Message #{self.message_count} ---')
            self.get_logger().debug(f'Incoming real joint names: {msg.name}')
            
            # Create a dictionary for easy lookup of real joint data
            real_joint_dict = {}
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    real_joint_dict[joint_name] = {
                        'position': msg.position[i],
                        'velocity': msg.velocity[i] if len(msg.velocity) > i else 0.0
                    }
            
            self.get_logger().debug(f'Real joint dict: {len(real_joint_dict)} joints')
            
            # Create ordered lists of positions and velocities for the simulation
            sim_positions = []
            sim_velocities = []
            missing_joints = []
            
            # Iterate through the simulation's expected joint order
            for sim_joint_name in self.sim_joint_names:
                # Find the corresponding real joint name using the reverse mapping
                real_joint_name = self.reverse_mapping.get(sim_joint_name)
                
                if real_joint_name and real_joint_name in real_joint_dict:
                    pos = real_joint_dict[real_joint_name]['position']
                    vel = real_joint_dict[real_joint_name]['velocity']
                    sim_positions.append(pos)
                    sim_velocities.append(vel)
                    self.get_logger().debug(f'Mapped: {real_joint_name} -> {sim_joint_name}, pos={pos:.3f}')
                else:
                    missing_joints.append(sim_joint_name)
                    sim_positions.append(0.0)  # Default value
                    sim_velocities.append(0.0)
            
            if missing_joints:
                self.get_logger().warn(f'Missing joints (using 0.0): {missing_joints}')
                self.get_logger().warn(f'Available real joints: {list(real_joint_dict.keys())}')
            
            # Rate limiting check
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_command_time).nanoseconds / 1e9
            
            if time_diff < self.command_rate_limit:
                self.get_logger().debug(f'Rate limit: skip ({time_diff:.3f}s)')
                return
                
            self.last_command_time = current_time
            self.last_real_positions = sim_positions
            self.last_real_velocities = sim_velocities
            
            # Send command to Gazebo
            success = self.send_to_gazebo(sim_positions, sim_velocities)
            
            if success:
                self.success_count += 1
                if self.message_count % 50 == 0:
                    self.get_logger().info(f'✓ Message #{self.message_count} (success: {self.success_count})')
            else:
                self.error_count += 1
                self.get_logger().error(f'✗ Message #{self.message_count} failed to send')
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Real joint callback error #{self.message_count}: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def send_to_gazebo(self, positions, velocities):
        """Sends positions as JointTrajectory to Gazebo"""
        try:
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = self.sim_joint_names
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.time_from_start = Duration(sec=0, nanosec=int(0.1 * 1e9))
            
            traj_msg.points = [point]
            
            sub_count = self.sim_joint_pub.get_subscription_count()
            if sub_count == 0:
                if self.message_count % 100 == 0:
                    self.get_logger().warn('⚠️  Controller is not listening! Is the topic correct?')
            else:
                self.get_logger().debug(f'✓ {sub_count} subscribers found')
            
            self.sim_joint_pub.publish(traj_msg)
            
            if self.message_count <= 5 or self.message_count % 50 == 0:
                self.get_logger().info(f'📤 Message #{self.message_count} sent.')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error sending to Gazebo: {str(e)}')
            return False

    def sim_joint_callback(self, msg):
        """Listens to simulation joint states for debugging"""
        try:
            if self.get_logger().get_effective_level() <= 10:
                self.get_logger().debug(f'📥 Sim joints: {[round(p, 3) for p in msg.position]}')
                
                if self.last_real_positions and len(msg.position) >= len(self.sim_joint_names):
                    max_diff = 0
                    for i, sim_joint_name in enumerate(self.sim_joint_names):
                        if sim_joint_name in msg.name:
                            sim_idx = msg.name.index(sim_joint_name)
                            sim_pos = msg.position[sim_idx]
                            real_pos = self.last_real_positions[i]
                            diff = abs(sim_pos - real_pos)
                            max_diff = max(max_diff, diff)
                    
                    if max_diff > 0.1:
                        self.get_logger().warn(f'🔍 Large position difference detected: max_diff={max_diff:.3f} rad')
                        
        except Exception as e:
            self.get_logger().debug(f'Sim joint callback error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = RealToSimJointSync()
        
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info('🤖 Node is running... Press Ctrl+C to stop.')
        executor.spin()
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('👋 Stopped by user.')
    except Exception as e:
        if node:
            node.get_logger().error(f'💥 Node error: {str(e)}')
        else:
            print(f'💥 Node creation error: {str(e)}')
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()