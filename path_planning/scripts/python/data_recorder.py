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
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
plt.ioff()  # Turn off interactive mode
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import tf2_ros
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState


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
        self.recorded_ee_poses = []  # Store end-effector poses
        self.recording_lock = threading.Lock()
        
        # TF buffer for getting end-effector position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # MoveIt service clients 
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
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
        
        # Timer for getting EE position during recording
        self.ee_timer = self.create_timer(0.01, self.get_ee_pose)  # 100Hz
        self.ee_timer.cancel()  # Start disabled
        
        self.get_logger().info('Data recorder ready. Use services to start/stop recording.')
    
    def joint_state_callback(self, msg):
        with self.recording_lock:
            if self.recording:
                self.recorded_states.append(msg)
                
                # Log every 100 messages
                if len(self.recorded_states) % 100 == 0:
                    self.get_logger().info(f'Recorded {len(self.recorded_states)} joint states')
    
    def get_ee_pose(self):
        """Get end-effector pose using TF"""
        if not self.recording:
            return
            
        try:
            # Get transform from world to EE (wrist_3_link is the last link in UR5)
            transform = self.tf_buffer.lookup_transform(
                'world', 
                'needle',
                rclpy.time.Time()
            )
            
            # Store in recorded_ee_poses
            with self.recording_lock:
                self.recorded_ee_poses.append(transform)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Failed to get EE transform: {e}')
    
    def start_recording_callback(self, request, response):
        with self.recording_lock:
            self.recorded_states.clear()
            self.recorded_ee_poses.clear()
            self.recording = True
            
            # Store planning information
            self.planning_time = request.planning_time
            self.target_frame = request.target_frame
            
            # Start the EE timer
            self.ee_timer.reset()
            
            self.get_logger().info(f'Started recording joint states for target {self.target_frame}. Planning time: {self.planning_time:.3f}s')
            
        response.success = True
        response.message = 'Recording started'
        return response
    
    def stop_recording_callback(self, request, response):
        states_copy = []
        ee_poses_copy = []
        planning_time = 0.0
        target_frame = ""
        
        with self.recording_lock:
            self.recording = False
            states_copy = list(self.recorded_states)
            ee_poses_copy = list(self.recorded_ee_poses)
            planning_time = self.planning_time
            target_frame = self.target_frame
            
            # Stop the EE timer
            self.ee_timer.cancel()
        
        # Process and visualize the recorded data
        if states_copy:
            timestamp = self.get_current_timestamp()
            # Process the data first
            processed_data = self.process_joint_data(states_copy, planning_time, target_frame, timestamp)
            
            # Process EE data
            processed_ee_data = self.process_ee_data(ee_poses_copy, planning_time, target_frame, timestamp)
            
            # Visualize the data if enabled
            if self.show_plots:
                self.visualize_joint_data(processed_data)
                if processed_ee_data:
                    self.visualize_ee_data(processed_ee_data)
            
            self.get_logger().info(f'Stopped recording and processed {len(states_copy)} joint states and {len(ee_poses_copy)} EE poses')
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
    
    def process_ee_data(self, ee_poses, planning_time, target_frame, timestamp):
        """Process EE pose data to extract positions, velocities, and accelerations"""
        if not ee_poses or len(ee_poses) < 2:
            self.get_logger().warn('Not enough EE poses recorded to process data')
            return None
        
        try:
            # Get the start time from the first message
            first_msg = ee_poses[0]
            start_sec = first_msg.header.stamp.sec
            start_nsec = first_msg.header.stamp.nanosec
            
            # Initialize data structures
            timestamps = []
            positions_x = []
            positions_y = []
            positions_z = []
            velocities_x = []
            velocities_y = []
            velocities_z = []
            accelerations_x = []
            accelerations_y = []
            accelerations_z = []
            
            # Extract timestamps and positions
            for i, pose in enumerate(ee_poses):
                # Calculate time relative to the first message in seconds
                time_secs = (pose.header.stamp.sec - start_sec) + ((pose.header.stamp.nanosec - start_nsec) * 1e-9)
                # Handle negative time that might occur due to precision errors
                if time_secs < 0:
                    time_secs = 0.0
                timestamps.append(time_secs)
                
                # Extract positions
                positions_x.append(pose.transform.translation.x)
                positions_y.append(pose.transform.translation.y)
                positions_z.append(pose.transform.translation.z)
            
            # Calculate velocities using central differentiation for smoother results
            # First point has zero velocity
            velocities_x.append(0.0)
            velocities_y.append(0.0)
            velocities_z.append(0.0)
            
            # Calculate velocities for middle points using central difference
            for i in range(1, len(timestamps)-1):
                # Use central difference for interior points
                dt_forward = timestamps[i+1] - timestamps[i]
                dt_backward = timestamps[i] - timestamps[i-1]
                dt_total = timestamps[i+1] - timestamps[i-1]
                
                if dt_total > 0.0:
                    # For position-based velocity, use central difference for smoother results
                    vx = (positions_x[i+1] - positions_x[i-1]) / dt_total
                    vy = (positions_y[i+1] - positions_y[i-1]) / dt_total
                    vz = (positions_z[i+1] - positions_z[i-1]) / dt_total
                    
                    velocities_x.append(vx)
                    velocities_y.append(vy)
                    velocities_z.append(vz)
                else:
                    # If time difference is too small, use the previous velocity
                    velocities_x.append(velocities_x[-1])
                    velocities_y.append(velocities_y[-1])
                    velocities_z.append(velocities_z[-1])
            
            # Last point velocity (forward difference)
            if len(timestamps) > 1:
                i = len(timestamps)-1
                dt = timestamps[i] - timestamps[i-1]
                if dt > 0:
                    vx = (positions_x[i] - positions_x[i-1]) / dt
                    vy = (positions_y[i] - positions_y[i-1]) / dt
                    vz = (positions_z[i] - positions_z[i-1]) / dt
                    velocities_x.append(vx)
                    velocities_y.append(vy)
                    velocities_z.append(vz)
                else:
                    # Use the previous velocity
                    velocities_x.append(velocities_x[-1])
                    velocities_y.append(velocities_y[-1])
                    velocities_z.append(velocities_z[-1])
            
            # Apply a simple moving average filter to smooth the velocities
            window_size = 5
            if len(velocities_x) >= window_size:
                smoothed_vx = np.convolve(velocities_x, np.ones(window_size)/window_size, mode='valid')
                smoothed_vy = np.convolve(velocities_y, np.ones(window_size)/window_size, mode='valid')
                smoothed_vz = np.convolve(velocities_z, np.ones(window_size)/window_size, mode='valid')
                
                # Pad the beginning with the first value
                pad_begin = [smoothed_vx[0]] * (window_size//2)
                velocities_x = np.concatenate([pad_begin, smoothed_vx])
                
                pad_begin = [smoothed_vy[0]] * (window_size//2)
                velocities_y = np.concatenate([pad_begin, smoothed_vy])
                
                pad_begin = [smoothed_vz[0]] * (window_size//2)
                velocities_z = np.concatenate([pad_begin, smoothed_vz])
                
                # Pad the end if necessary
                if len(velocities_x) < len(timestamps):
                    pad_end = [smoothed_vx[-1]] * (len(timestamps) - len(velocities_x))
                    velocities_x = np.concatenate([velocities_x, pad_end])
                    velocities_y = np.concatenate([velocities_y, pad_end])
                    velocities_z = np.concatenate([velocities_z, pad_end])
            
            # Calculate accelerations using central difference
            # First point has zero acceleration
            accelerations_x.append(0.0)
            accelerations_y.append(0.0)
            accelerations_z.append(0.0)
            
            # Calculate accelerations for middle points
            for i in range(1, len(timestamps)-1):
                dt_total = timestamps[i+1] - timestamps[i-1]
                if dt_total > 0:
                    ax = (velocities_x[i+1] - velocities_x[i-1]) / dt_total
                    ay = (velocities_y[i+1] - velocities_y[i-1]) / dt_total
                    az = (velocities_z[i+1] - velocities_z[i-1]) / dt_total
                    accelerations_x.append(ax)
                    accelerations_y.append(ay)
                    accelerations_z.append(az)
                else:
                    # Use previous value if time difference is too small
                    accelerations_x.append(accelerations_x[-1])
                    accelerations_y.append(accelerations_y[-1])
                    accelerations_z.append(accelerations_z[-1])
            
            # Last point acceleration (use previous)
            accelerations_x.append(accelerations_x[-1])
            accelerations_y.append(accelerations_y[-1])
            accelerations_z.append(accelerations_z[-1])
            
            # Apply a moving average filter to smooth the accelerations
            if len(accelerations_x) >= window_size:
                smoothed_ax = np.convolve(accelerations_x, np.ones(window_size)/window_size, mode='valid')
                smoothed_ay = np.convolve(accelerations_y, np.ones(window_size)/window_size, mode='valid')
                smoothed_az = np.convolve(accelerations_z, np.ones(window_size)/window_size, mode='valid')
                
                # Pad the beginning
                pad_begin = [smoothed_ax[0]] * (window_size//2)
                accelerations_x = np.concatenate([pad_begin, smoothed_ax])
                
                pad_begin = [smoothed_ay[0]] * (window_size//2)
                accelerations_y = np.concatenate([pad_begin, smoothed_ay])
                
                pad_begin = [smoothed_az[0]] * (window_size//2)
                accelerations_z = np.concatenate([pad_begin, smoothed_az])
                
                # Pad the end if necessary
                if len(accelerations_x) < len(timestamps):
                    pad_end = [smoothed_ax[-1]] * (len(timestamps) - len(accelerations_x))
                    accelerations_x = np.concatenate([accelerations_x, pad_end])
                    accelerations_y = np.concatenate([accelerations_y, pad_end])
                    accelerations_z = np.concatenate([accelerations_z, pad_end])
            
            # Get target pose
            target_pose = None
            try:
                target_transform = self.tf_buffer.lookup_transform(
                    'world', 
                    target_frame,
                    rclpy.time.Time()
                )
                target_pose = {
                    'x': target_transform.transform.translation.x,
                    'y': target_transform.transform.translation.y,
                    'z': target_transform.transform.translation.z
                }
            except Exception as e:
                self.get_logger().warn(f'Could not get target pose: {e}')
            
            # Return processed data
            return {
                'timestamp': timestamp,
                'target_frame': target_frame,
                'planning_time': planning_time,
                'timestamps': timestamps,
                'positions_x': positions_x,
                'positions_y': positions_y,
                'positions_z': positions_z,
                'velocities_x': velocities_x,
                'velocities_y': velocities_y,
                'velocities_z': velocities_z,
                'accelerations_x': accelerations_x,
                'accelerations_y': accelerations_y,
                'accelerations_z': accelerations_z,
                'target_pose': target_pose,
                'total_points': len(timestamps)
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to process EE data: {str(e)}')
            return None
    
    def visualize_joint_data(self, data):
        """Visualize the processed joint data using matplotlib"""
        if not data:
            self.get_logger().warn('No processed data available to visualize')
            return
        
        try:
            # Create the figure with gridspec for better layout
            fig = plt.figure(figsize=(15, 10), constrained_layout=True)
            gs = GridSpec(2, 2, figure=fig)  # 2 rows, 2 columns
            
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
            
            # Save the figure
            plt_filename = f'trajectory_analysis_{target_name}_{data["timestamp"]}.png'
            plt.savefig(plt_filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Saved visualization to {plt_filename}')
            
            # Close the figure to avoid memory leaks and hanging windows
            plt.close(fig)
        
        except Exception as e:
            self.get_logger().error(f'Failed to visualize joint data: {str(e)}')
    
    def visualize_ee_data(self, data):
        """Visualize end-effector trajectory in 3D and position/velocity/acceleration graphs"""
        if not data:
            self.get_logger().warn('No processed end-effector data available to visualize')
            return
        
        try:
            # Create a new figure for EE visualization
            fig = plt.figure(figsize=(15, 10), constrained_layout=True)
            gs = GridSpec(2, 2, figure=fig)
            
            target_name = data['target_frame'].split('/')[-1] if '/' in data['target_frame'] else data['target_frame']
            
            # Add title with metadata
            plt.suptitle(f'End-Effector Trajectory Analysis\n' + 
                        f'Target: {data["target_frame"]}\n' +
                        f'Planning Time: {data["planning_time"]:.3f}s, Trajectory Points: {data["total_points"]}',
                        fontsize=16)
            
            # 3D trajectory plot - top left
            ax_3d = fig.add_subplot(gs[0, 0], projection='3d')
            ax_3d.plot(data['positions_x'], data['positions_y'], data['positions_z'], 'b-', linewidth=2)
            ax_3d.scatter(data['positions_x'][0], data['positions_y'][0], data['positions_z'][0], 
                          color='g', s=100, label='Start')
            ax_3d.scatter(data['positions_x'][-1], data['positions_y'][-1], data['positions_z'][-1], 
                          color='r', s=100, label='End')
            
            # Plot target position if available
            if data['target_pose']:
                ax_3d.scatter(data['target_pose']['x'], data['target_pose']['y'], data['target_pose']['z'],
                            color='m', s=100, label='Target')
            
            ax_3d.set_title('End-Effector 3D Trajectory')
            ax_3d.set_xlabel('X (m)')
            ax_3d.set_ylabel('Y (m)')
            ax_3d.set_zlabel('Z (m)')
            ax_3d.legend()
            
            # Make axis equal for better visualization
            max_range = max([
                max(data['positions_x']) - min(data['positions_x']),
                max(data['positions_y']) - min(data['positions_y']),
                max(data['positions_z']) - min(data['positions_z'])
            ])
            mid_x = (max(data['positions_x']) + min(data['positions_x'])) / 2
            mid_y = (max(data['positions_y']) + min(data['positions_y'])) / 2
            mid_z = (max(data['positions_z']) + min(data['positions_z'])) / 2
            ax_3d.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
            ax_3d.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
            ax_3d.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
            
            # Position plot with target comparison - top right
            ax_pos = fig.add_subplot(gs[0, 1])
            ax_pos.plot(data['timestamps'], data['positions_x'], 'r-', label='X')
            ax_pos.plot(data['timestamps'], data['positions_y'], 'g-', label='Y')
            ax_pos.plot(data['timestamps'], data['positions_z'], 'b-', label='Z')
            
            # Add target position lines if available
            if data['target_pose']:
                ax_pos.axhline(y=data['target_pose']['x'], color='r', linestyle='--', alpha=0.5)
                ax_pos.axhline(y=data['target_pose']['y'], color='g', linestyle='--', alpha=0.5)
                ax_pos.axhline(y=data['target_pose']['z'], color='b', linestyle='--', alpha=0.5)
            
            ax_pos.set_title('End-Effector Position')
            ax_pos.set_xlabel('Time (s)')
            ax_pos.set_ylabel('Position (m)')
            ax_pos.grid(True)
            ax_pos.legend()
            
            # Velocity plot - bottom left
            ax_vel = fig.add_subplot(gs[1, 0])
            ax_vel.plot(data['timestamps'], data['velocities_x'], 'r-', label='X')
            ax_vel.plot(data['timestamps'], data['velocities_y'], 'g-', label='Y')
            ax_vel.plot(data['timestamps'], data['velocities_z'], 'b-', label='Z')
            ax_vel.set_title('End-Effector Velocity')
            ax_vel.set_xlabel('Time (s)')
            ax_vel.set_ylabel('Velocity (m/s)')
            ax_vel.grid(True)
            ax_vel.legend()
            
            # Acceleration plot - bottom right
            ax_acc = fig.add_subplot(gs[1, 1])
            ax_acc.plot(data['timestamps'], data['accelerations_x'], 'r-', label='X')
            ax_acc.plot(data['timestamps'], data['accelerations_y'], 'g-', label='Y')
            ax_acc.plot(data['timestamps'], data['accelerations_z'], 'b-', label='Z')
            ax_acc.set_title('End-Effector Acceleration')
            ax_acc.set_xlabel('Time (s)')
            ax_acc.set_ylabel('Acceleration (m/s²)')
            ax_acc.grid(True)
            ax_acc.legend()
            
            # Save the figure
            ee_plt_filename = f'ee_trajectory_analysis_{target_name}_{data["timestamp"]}.png'
            plt.savefig(ee_plt_filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Saved EE visualization to {ee_plt_filename}')
            
            # Close the figure to avoid memory leaks and hanging windows
            plt.close(fig)
            
        except Exception as e:
            self.get_logger().error(f'Failed to visualize end-effector data: {str(e)}')


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