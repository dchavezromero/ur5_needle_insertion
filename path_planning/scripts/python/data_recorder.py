#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import threading
import datetime
import os
import numpy as np
from builtin_interfaces.msg import Time
from path_planning.srv import StartRecording
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Declare parameters if they don't already exist
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception as e:
            self.get_logger().info(f'Parameter use_sim_time already declared: {e}')
        
        # State variables
        self.recording = False
        self.recorded_states = []
        self.recording_lock = threading.Lock()
        
        # Planning metrics
        self.planning_time = 0.0
        self.target_frame = ""
        
        # Visualization parameters
        self.declare_parameter('show_plots', True)
        self.show_plots = self.get_parameter('show_plots').value
        
        # Get use_sim_time parameter
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'Data Recorder Node Initialized with use_sim_time={use_sim_time}')
        self.get_logger().info(f'Plot visualization is {"enabled" if self.show_plots else "disabled"}')
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a subscription to joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )
        
        # Create services
        self.start_recording_service = self.create_service(
            StartRecording,
            'start_recording',
            self.start_recording_callback
        )
        
        self.stop_recording_service = self.create_service(
            Trigger,
            'stop_recording',
            self.stop_recording_callback
        )
        
        self.get_logger().info('Data recorder ready. Use services to start/stop recording.')
    
    def joint_state_callback(self, msg):
        with self.recording_lock:
            if self.recording:
                self.recorded_states.append(msg)
                
                # Log every 100 messages
                if len(self.recorded_states) % 100 == 0:
                    self.get_logger().info(f'Recorded {len(self.recorded_states)} joint states')
    
    def start_recording_callback(self, request, response):
        with self.recording_lock:
            self.recorded_states.clear()
            self.recording = True
            
            # Store planning information
            self.planning_time = request.planning_time
            self.target_frame = request.target_frame
            
            self.get_logger().info(f'Started recording joint states for target {self.target_frame}. Planning time: {self.planning_time:.3f}s')
            
        response.success = True
        response.message = 'Recording started'
        return response
    
    def stop_recording_callback(self, request, response):
        states_copy = []
        planning_time = 0.0
        target_frame = ""
        
        with self.recording_lock:
            self.recording = False
            states_copy = list(self.recorded_states)
            planning_time = self.planning_time
            target_frame = self.target_frame
        
        # Process and visualize the recorded data
        if states_copy:
            timestamp = self.get_current_timestamp()
            # Process the data first
            processed_data = self.process_joint_data(states_copy, planning_time, target_frame, timestamp)
            
            # Visualize the data if enabled
            if self.show_plots:
                self.visualize_joint_data(processed_data)
            
            self.get_logger().info(f'Stopped recording and processed {len(states_copy)} joint states')
            response.success = True
            response.message = 'Recording stopped and data processed'
        else:
            self.get_logger().warn('No joint states recorded, no data to process')
            response.success = True
            response.message = 'Recording stopped but no data was recorded'
        
        return response
    
    def get_current_timestamp(self):
        now = datetime.datetime.now()
        return now.strftime("%Y%m%d_%H%M%S")
    
    def process_joint_data(self, joint_states, planning_time, target_frame, timestamp):
        """Process the joint data to extract positions, velocities, and accelerations"""
        if not joint_states or len(joint_states) < 2:
            self.get_logger().warn('Not enough joint states recorded to process data')
            return None
        
        # List of joints we're interested in
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        try:
            # Check if velocities are available in the messages
            has_velocity_data = False
            first_state = joint_states[0]
            
            if hasattr(first_state, 'velocity') and len(first_state.velocity) > 0:
                has_velocity_data = True
            
            # Get indices of the joints we're interested in
            joint_indices = []
            for joint_name in joint_names:
                try:
                    joint_indices.append(first_state.name.index(joint_name))
                except ValueError:
                    self.get_logger().error(f'Joint {joint_name} not found in recorded joint states')
                    return None
            
            # Get the start time from the first message
            first_msg = joint_states[0]
            start_sec = first_msg.header.stamp.sec
            start_nsec = first_msg.header.stamp.nanosec
            
            # Initialize data structures
            timestamps = []
            positions = [[] for _ in range(len(joint_indices))]
            velocities = [[] for _ in range(len(joint_indices))]
            accelerations = [[] for _ in range(len(joint_indices))]
            
            # Extract timestamps and positions
            for state in joint_states:
                # Calculate time relative to the first message in seconds
                time_secs = (state.header.stamp.sec - start_sec) + ((state.header.stamp.nanosec - start_nsec) * 1e-9)
                # Handle negative time that might occur due to precision errors
                if time_secs < 0:
                    time_secs = 0.0
                timestamps.append(time_secs)
                
                # Extract positions
                for i, idx in enumerate(joint_indices):
                    if idx < len(state.position):
                        positions[i].append(state.position[idx])
                    else:
                        positions[i].append(float('nan'))
            
            # Calculate or extract velocities
            if has_velocity_data:
                # Extract velocities from messages
                for i, state in enumerate(joint_states):
                    for j, idx in enumerate(joint_indices):
                        if idx < len(state.velocity):
                            velocities[j].append(state.velocity[idx])
                        else:
                            velocities[j].append(0.0)
            else:
                # Calculate velocities from position differences
                for j in range(len(joint_indices)):
                    # First point has zero velocity
                    velocities[j].append(0.0)
                    
                    # Calculate velocities for remaining points
                    for i in range(1, len(timestamps)):
                        dt = timestamps[i] - timestamps[i-1]
                        if dt > 0 and not np.isnan(positions[j][i]) and not np.isnan(positions[j][i-1]):
                            vel = (positions[j][i] - positions[j][i-1]) / dt
                            velocities[j].append(vel)
                        else:
                            velocities[j].append(0.0)
            
            # Calculate accelerations from velocity differences
            for j in range(len(joint_indices)):
                # First point has zero acceleration
                accelerations[j].append(0.0)
                
                # Calculate accelerations for remaining points
                for i in range(1, len(timestamps)):
                    dt = timestamps[i] - timestamps[i-1]
                    if dt > 0:
                        accel = (velocities[j][i] - velocities[j][i-1]) / dt
                        accelerations[j].append(accel)
                    else:
                        accelerations[j].append(0.0)
            
            # Return processed data
            return {
                'timestamp': timestamp,
                'target_frame': target_frame,
                'planning_time': planning_time,
                'joint_names': joint_names,
                'timestamps': timestamps,
                'positions': positions,
                'velocities': velocities,
                'accelerations': accelerations,
                'total_points': len(timestamps)
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to process joint data: {str(e)}')
            return None
    
    def visualize_joint_data(self, data):
        """Visualize the processed joint data using matplotlib"""
        if not data:
            self.get_logger().warn('No processed data available to visualize')
            return
        
        try:
            # Create the figure with gridspec for better layout
            plt.figure(figsize=(15, 10), constrained_layout=True)
            gs = GridSpec(2, 2, figure=plt.gcf())  # 2 rows, 2 columns
            
            # Add title with metadata
            target_name = data['target_frame'].split('/')[-1] if '/' in data['target_frame'] else data['target_frame']
            
            # Display planning time and target frame in the title
            plt.suptitle(f'Joint Trajectory Analysis\n' + 
                         f'Target: {data["target_frame"]}\n' +
                         f'Planning Time: {data["planning_time"]:.3f}s, Trajectory Points: {data["total_points"]}',
                         fontsize=16)
            
            # Color palette for joints
            colors = ['b', 'g', 'r', 'c', 'm', 'y']
            
            # Plot positions - left column, top row
            ax_pos = plt.subplot(gs[0, 0])
            for i, joint_name in enumerate(data['joint_names']):
                ax_pos.plot(data['timestamps'], data['positions'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_pos.set_title('Joint Positions')
            ax_pos.set_xlabel('Time (s)')
            ax_pos.set_ylabel('Position (rad)')
            ax_pos.grid(True)
            ax_pos.legend(loc='best')
            
            # Plot velocities - right column, top row
            ax_vel = plt.subplot(gs[0, 1])
            for i, joint_name in enumerate(data['joint_names']):
                ax_vel.plot(data['timestamps'], data['velocities'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_vel.set_title('Joint Velocities')
            ax_vel.set_xlabel('Time (s)')
            ax_vel.set_ylabel('Velocity (rad/s)')
            ax_vel.grid(True)
            ax_vel.legend(loc='best')
            
            # Plot accelerations - left column, bottom row
            ax_acc = plt.subplot(gs[1, 0])
            for i, joint_name in enumerate(data['joint_names']):
                ax_acc.plot(data['timestamps'], data['accelerations'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_acc.set_title('Joint Accelerations')
            ax_acc.set_xlabel('Time (s)')
            ax_acc.set_ylabel('Acceleration (rad/s²)')
            ax_acc.grid(True)
            ax_acc.legend(loc='best')
            
            # Calculate and plot path smoothness metrics - right column, bottom row
            ax_metrics = plt.subplot(gs[1, 1])
            
            # Calculate metrics for each joint
            avg_velocities = [np.mean(np.abs(v)) for v in data['velocities']]
            max_velocities = [np.max(np.abs(v)) for v in data['velocities']]
            avg_accelerations = [np.mean(np.abs(a)) for a in data['accelerations']]
            max_accelerations = [np.max(np.abs(a)) for a in data['accelerations']]
            
            # Create bar chart for metrics
            x = np.arange(len(data['joint_names']))
            width = 0.2
            
            ax_metrics.bar(x - 1.5*width, avg_velocities, width, color='b', label='Avg Vel')
            ax_metrics.bar(x - 0.5*width, max_velocities, width, color='g', label='Max Vel')
            ax_metrics.bar(x + 0.5*width, avg_accelerations, width, color='r', label='Avg Accel')
            ax_metrics.bar(x + 1.5*width, max_accelerations, width, color='c', label='Max Accel')
            
            ax_metrics.set_title('Motion Smoothness Metrics')
            ax_metrics.set_xticks(x)
            ax_metrics.set_xticklabels([name.split('_')[0] for name in data['joint_names']], rotation=45)
            ax_metrics.set_ylabel('Value')
            ax_metrics.grid(True, axis='y')
            ax_metrics.legend(loc='best')
            
            # Add text with planning information
            plt.figtext(0.02, 0.02, f"Planning Time: {data['planning_time']:.3f}s | Target: {data['target_frame']}", 
                       fontsize=12, weight='bold', bbox=dict(facecolor='white', alpha=0.8))
            
            # Save the figure
            plt_filename = f'trajectory_analysis_{target_name}_{data["timestamp"]}.png'
            plt.savefig(plt_filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Saved visualization to {plt_filename}')
            
            # Show the plot if running in an environment with display
            plt.show()
            
        except Exception as e:
            self.get_logger().error(f'Failed to visualize joint data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 