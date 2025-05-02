#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import datetime
import os
import threading

# ROS message imports
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from path_planning.srv import StartRecording
import tf2_ros

# Visualization imports
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
plt.ioff()  # Turn off interactive mode
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D


class DataRecorder(Node):
    """
    Records joint trajectories and end-effector movement data for analysis.
    Provides services to start/stop recording and generates visualizations.
    """
    def __init__(self):
        super().__init__('data_recorder')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize state variables
        self._init_state()
        
        # Set up TF for getting end-effector position
        self._setup_tf()
        
        # Set up subscribers, services, and timers
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()
        
        self.get_logger().info('Data recorder ready. Use services to start/stop recording.')
    
    def _init_parameters(self):
        """Initialize node parameters"""
        # Declare parameters if they don't already exist
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception as e:
            self.get_logger().info(f'Parameter use_sim_time already declared: {e}')
        
        # Visualization parameters
        self.declare_parameter('show_plots', True)
        self.show_plots = self.get_parameter('show_plots').value
        
        # Get use_sim_time parameter
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'Data Recorder Node Initialized with use_sim_time={use_sim_time}')
        self.get_logger().info(f'Plot visualization is {"enabled" if self.show_plots else "disabled"}')
    
    def _init_state(self):
        """Initialize state variables for recording data"""
        self.recording = False
        self.recorded_states = []
        self.recorded_ee_poses = []
        self.recorded_ee_velocities = []
        self.ee_velocity_count = 0
        self.recording_lock = threading.Lock()
        
        # Planning metrics
        self.planning_time = 0.0
        self.target_frame = ""
        self.current_planner = "Unknown"
    
    def _setup_tf(self):
        """Set up TF buffer for transforms"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def _setup_subscribers(self):
        """Set up subscribers for joint states and EE velocity"""
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
        
        # Create a subscription to ee_velocity
        self.ee_velocity_sub = self.create_subscription(
            TwistStamped,
            '/ee_velocity',
            self.ee_velocity_callback,
            sensor_qos
        )
    
    def _setup_services(self):
        """Set up services for starting and stopping recording"""
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
    
    def _setup_timers(self):
        """Set up timers for periodic operations"""
        # Timer for getting EE position during recording (200Hz)
        self.ee_timer = self.create_timer(0.005, self.get_ee_pose)
        self.ee_timer.cancel()  # Start disabled

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        with self.recording_lock:
            if self.recording:
                self.recorded_states.append(msg)
                
                # Log every 100 messages
                if len(self.recorded_states) % 100 == 0:
                    self.get_logger().info(f'Recorded {len(self.recorded_states)} joint states')
    
    def ee_velocity_callback(self, msg):
        """Callback for end-effector velocity messages"""
        with self.recording_lock:
            if self.recording:
                # Store the velocity
                self.recorded_ee_velocities.append(msg)
                
                # Count for logging
                self.ee_velocity_count += 1
                if self.ee_velocity_count % 100 == 0:
                    self.get_logger().info(f'Received {self.ee_velocity_count} EE velocity messages')
    
    def get_ee_pose(self):
        """Get end-effector pose using TF"""
        if not self.recording:
            return
            
        try:
            # Get transform from world to needle
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
        """Service callback to start recording data"""
        with self.recording_lock:
            # Clear previous data
            self.recorded_states.clear()
            self.recorded_ee_poses.clear()
            self.recorded_ee_velocities.clear()
            self.ee_velocity_count = 0
            
            # Start recording
            self.recording = True
            
            # Store planning information
            self.planning_time = request.planning_time
            self.target_frame = request.target_frame
            self.current_planner = request.planning_algorithm if hasattr(request, 'planning_algorithm') else "Unknown"
            
            # Start the EE timer
            self.ee_timer.reset()
            
            self.get_logger().info(f'Started recording joint states for target {self.target_frame}. '
                                   f'Planning time: {self.planning_time:.3f}s, Planner: {self.current_planner}')
            
        response.success = True
        response.message = 'Recording started'
        return response
    
    def stop_recording_callback(self, request, response):
        """Service callback to stop recording and process data"""
        states_copy = []
        ee_poses_copy = []
        ee_velocities_copy = []
        planning_time = 0.0
        target_frame = ""
        current_planner = ""
        
        with self.recording_lock:
            # Stop recording
            self.recording = False
            
            # Make copies of data for processing
            states_copy = list(self.recorded_states)
            ee_poses_copy = list(self.recorded_ee_poses)
            ee_velocities_copy = list(self.recorded_ee_velocities)
            
            # Get planning info
            planning_time = self.planning_time
            target_frame = self.target_frame
            current_planner = self.current_planner
            
            # Stop the EE timer
            self.ee_timer.cancel()
        
        # Process and visualize the recorded data
        if states_copy:
            timestamp = self._get_current_timestamp()
            
            # Process the joint and EE data
            processed_joint_data = self._process_joint_data(states_copy, planning_time, target_frame, current_planner, timestamp)
            processed_ee_data = self._process_ee_data(ee_poses_copy, ee_velocities_copy, planning_time, target_frame, current_planner, timestamp)
            
            # Generate visualizations if enabled
            if self.show_plots:
                if processed_joint_data:
                    self._visualize_joint_data(processed_joint_data)
                if processed_ee_data:
                    self._visualize_ee_data(processed_ee_data)
            
            self.get_logger().info(f'Stopped recording and processed {len(states_copy)} joint states, '
                                   f'{len(ee_poses_copy)} EE poses, and {len(ee_velocities_copy)} EE velocity messages')
            response.success = True
            response.message = 'Recording stopped and data processed'
        else:
            self.get_logger().warn('No joint states recorded, no data to process')
            response.success = True
            response.message = 'Recording stopped but no data was recorded'
        
        return response
    
    def _get_current_timestamp(self):
        """Get formatted timestamp string for filenames"""
        now = datetime.datetime.now()
        return now.strftime("%Y%m%d_%H%M%S")
    
    def _process_joint_data(self, joint_states, planning_time, target_frame, planner_name, timestamp):
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
            # Check first joint state message
            first_state = joint_states[0]
            has_velocity_data = hasattr(first_state, 'velocity') and len(first_state.velocity) > 0
            
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
                time_secs = max(0.0, time_secs)  # Handle negative time due to precision errors
                timestamps.append(time_secs)
                
                # Extract positions
                for i, idx in enumerate(joint_indices):
                    if idx < len(state.position):
                        positions[i].append(state.position[idx])
                    else:
                        positions[i].append(float('nan'))
            
            # Calculate or extract velocities
            for j in range(len(joint_indices)):
                # Extract or calculate velocities
                if has_velocity_data:
                    # Extract from messages
                    for state in joint_states:
                        idx = joint_indices[j]
                        if idx < len(state.velocity):
                            velocities[j].append(state.velocity[idx])
                        else:
                            velocities[j].append(0.0)
                else:
                    # Calculate from positions (without smoothing)
                    self._calculate_velocities(positions[j], timestamps, velocities[j])
                
                # Calculate accelerations from velocities
                self._calculate_accelerations(velocities[j], timestamps, accelerations[j])
            
            # Return processed data
            return {
                'timestamp': timestamp,
                'target_frame': target_frame,
                'planning_time': planning_time,
                'planner': planner_name,
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
    
    def _process_ee_data(self, ee_poses, ee_velocities, planning_time, target_frame, planner_name, timestamp):
        """Process EE pose data to extract positions, velocities, and final position error"""
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
            
            # Extract timestamps and positions from all poses
            for pose in ee_poses:
                # Calculate time relative to the first message in seconds
                time_secs = (pose.header.stamp.sec - start_sec) + ((pose.header.stamp.nanosec - start_nsec) * 1e-9)
                time_secs = max(0.0, time_secs)  # Handle negative time due to precision errors
                timestamps.append(time_secs)
                
                # Extract positions
                positions_x.append(pose.transform.translation.x)
                positions_y.append(pose.transform.translation.y)
                positions_z.append(pose.transform.translation.z)
            
            # Process the velocity data from messages
            velocities_x, velocities_y, velocities_z = self._process_velocity_data(
                ee_velocities, timestamps, start_sec, start_nsec
            )
            
            # Get target pose
            target_pose = self._get_target_pose(target_frame)
            
            # Calculate final position and error
            final_pos = {
                'x': positions_x[-1] if positions_x else 0.0,
                'y': positions_y[-1] if positions_y else 0.0,
                'z': positions_z[-1] if positions_z else 0.0
            }
            
            position_error = self._calculate_position_error(final_pos, target_pose)
            
            self.get_logger().info(f"Final position: X={final_pos['x']:.4f}, Y={final_pos['y']:.4f}, Z={final_pos['z']:.4f}")
            self.get_logger().info(f"Position error: X={position_error['x']:.4f}, Y={position_error['y']:.4f}, "
                                   f"Z={position_error['z']:.4f}, Euclidean={position_error['euclidean']:.4f}")
            
            # Return processed data
            return {
                'timestamp': timestamp,
                'target_frame': target_frame,
                'planning_time': planning_time,
                'planner': planner_name,
                'timestamps': timestamps,
                'positions_x': positions_x,
                'positions_y': positions_y,
                'positions_z': positions_z,
                'velocities_x': velocities_x,
                'velocities_y': velocities_y,
                'velocities_z': velocities_z,
                'target_pose': target_pose,
                'final_position': final_pos,
                'position_error': position_error,
                'total_points': len(timestamps)
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to process EE data: {str(e)}')
            return None
    
    def _process_velocity_data(self, velocity_msgs, pose_timestamps, start_sec, start_nsec):
        """Process velocity messages without filtering - preserve raw data"""
        # Extract the raw velocity data
        raw_vel_times = []
        raw_vel_x = []
        raw_vel_y = []
        raw_vel_z = []
        
        for vel_msg in velocity_msgs:
            vel_time = (vel_msg.header.stamp.sec - start_sec) + ((vel_msg.header.stamp.nanosec - start_nsec) * 1e-9)
            vel_time = max(0.0, vel_time)  # Handle negative time
            
            raw_vel_times.append(vel_time)
            raw_vel_x.append(vel_msg.twist.linear.x)
            raw_vel_y.append(vel_msg.twist.linear.y)
            raw_vel_z.append(vel_msg.twist.linear.z)
        
        # Interpolate raw velocities at pose timestamps (no filtering)
        velocities_x = self._interpolate_at_timestamps(pose_timestamps, raw_vel_times, raw_vel_x)
        velocities_y = self._interpolate_at_timestamps(pose_timestamps, raw_vel_times, raw_vel_y)
        velocities_z = self._interpolate_at_timestamps(pose_timestamps, raw_vel_times, raw_vel_z)
        
        self.get_logger().info(f'Processed {len(raw_vel_times)} raw velocity measurements')
        return velocities_x, velocities_y, velocities_z
    
    def _visualize_joint_data(self, data):
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
            planner_name = data.get('planner', 'Unknown')
            
            plt.suptitle(f'Joint Trajectory Analysis - Planner: {planner_name}\n' + 
                         f'Target: {data["target_frame"]}\n' +
                         f'Planning Time: {data["planning_time"]:.3f}s, Trajectory Points: {data["total_points"]}',
                         fontsize=16)
            
            # Color palette for joints
            colors = ['b', 'g', 'r', 'c', 'm', 'y']
            
            # Plot positions - top left
            ax_pos = plt.subplot(gs[0, 0])
            for i, joint_name in enumerate(data['joint_names']):
                ax_pos.plot(data['timestamps'], data['positions'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_pos.set_title('Joint Positions')
            ax_pos.set_xlabel('Time (s)')
            ax_pos.set_ylabel('Position (rad)')
            ax_pos.grid(True)
            ax_pos.legend(loc='best')
            
            # Plot velocities - top right
            ax_vel = plt.subplot(gs[0, 1])
            for i, joint_name in enumerate(data['joint_names']):
                ax_vel.plot(data['timestamps'], data['velocities'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_vel.set_title('Joint Velocities')
            ax_vel.set_xlabel('Time (s)')
            ax_vel.set_ylabel('Velocity (rad/s)')
            ax_vel.grid(True)
            ax_vel.legend(loc='best')
            
            # Plot accelerations - bottom left
            ax_acc = plt.subplot(gs[1, 0])
            for i, joint_name in enumerate(data['joint_names']):
                ax_acc.plot(data['timestamps'], data['accelerations'][i], color=colors[i % len(colors)], 
                           label=joint_name)
            ax_acc.set_title('Joint Accelerations')
            ax_acc.set_xlabel('Time (s)')
            ax_acc.set_ylabel('Acceleration (rad/s²)')
            ax_acc.grid(True)
            ax_acc.legend(loc='best')
            
            # Calculate and plot path smoothness metrics - bottom right
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
            
            # Save the figure as SVG
            plt_filename = f'trajectory_analysis_{target_name}_{data["timestamp"]}.svg'
            plt.savefig(plt_filename, bbox_inches='tight')
            self.get_logger().info(f'Saved joint visualization to {plt_filename}')
            
            # Close the figure to avoid memory leaks
            plt.close(fig)
        
        except Exception as e:
            self.get_logger().error(f'Failed to visualize joint data: {str(e)}')
    
    def _visualize_ee_data(self, data):
        """Visualize end-effector trajectory in 3D and position/velocity/accuracy graphs"""
        if not data:
            self.get_logger().warn('No processed end-effector data available to visualize')
            return
        
        try:
            # Create a new figure for EE visualization
            fig = plt.figure(figsize=(15, 10), constrained_layout=True)
            gs = GridSpec(2, 2, figure=fig)
            
            target_name = data['target_frame'].split('/')[-1] if '/' in data['target_frame'] else data['target_frame']
            planner_name = data.get('planner', 'Unknown')
            
            # Add title with metadata
            plt.suptitle(f'End-Effector Trajectory Analysis - Planner: {planner_name}\n' + 
                        f'Target: {data["target_frame"]}\n' +
                        f'Planning Time: {data["planning_time"]:.3f}s, Trajectory Points: {data["total_points"]}',
                        fontsize=16)
            
            # 3D trajectory plot - top left
            self._plot_3d_trajectory(fig, gs[0, 0], data)
            
            # Position plot - top right
            self._plot_position_data(fig, gs[0, 1], data)
            
            # Velocity plot - bottom left
            self._plot_velocity_data(fig, gs[1, 0], data)
            
            # Position error - bottom right
            self._plot_position_error(fig, gs[1, 1], data)
            
            # Save the figure as SVG
            ee_plt_filename = f'ee_trajectory_analysis_{target_name}_{data["timestamp"]}.svg'
            plt.savefig(ee_plt_filename, bbox_inches='tight')
            self.get_logger().info(f'Saved EE visualization to {ee_plt_filename}')
            
            # Close the figure to avoid memory leaks
            plt.close(fig)
            
        except Exception as e:
            self.get_logger().error(f'Failed to visualize end-effector data: {str(e)}')
    
    def _plot_3d_trajectory(self, fig, gs_pos, data):
        """Plot 3D trajectory of end-effector"""
        ax_3d = fig.add_subplot(gs_pos, projection='3d')
        ax_3d.plot(data['positions_x'], data['positions_y'], data['positions_z'], 'b-', linewidth=2)
        ax_3d.scatter(data['positions_x'][0], data['positions_y'][0], data['positions_z'][0], 
                      color='g', s=100, label='Start')
        ax_3d.scatter(data['positions_x'][-1], data['positions_y'][-1], data['positions_z'][-1], 
                      color='r', s=100, label='End')
        
        # Plot target position if available
        if data['target_pose']:
            ax_3d.scatter(data['target_pose']['x'], data['target_pose']['y'], data['target_pose']['z'],
                        color='m', s=100, label='Target')
            
            # Draw line from end position to target
            ax_3d.plot([data['positions_x'][-1], data['target_pose']['x']],
                       [data['positions_y'][-1], data['target_pose']['y']],
                       [data['positions_z'][-1], data['target_pose']['z']],
                       'r--', linewidth=1.5, label='Position Error')
        
        ax_3d.set_title('End-Effector 3D Trajectory')
        ax_3d.set_xlabel('X (m)')
        ax_3d.set_ylabel('Y (m)')
        ax_3d.set_zlabel('Z (m)')
        ax_3d.legend()
        
        # Make axis equal for better visualization
        self._set_axes_equal_3d(ax_3d, data['positions_x'], data['positions_y'], data['positions_z'])
    
    def _set_axes_equal_3d(self, ax, x_data, y_data, z_data):
        """Make axes of 3D plot have equal scale"""
        max_range = max([
            max(x_data) - min(x_data),
            max(y_data) - min(y_data),
            max(z_data) - min(z_data)
        ])
        mid_x = (max(x_data) + min(x_data)) / 2
        mid_y = (max(y_data) + min(y_data)) / 2
        mid_z = (max(z_data) + min(z_data)) / 2
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
    
    def _plot_position_data(self, fig, gs_pos, data):
        """Plot position data with target comparison"""
        ax_pos = fig.add_subplot(gs_pos)
        ax_pos.plot(data['timestamps'], data['positions_x'], 'r-', label='X')
        ax_pos.plot(data['timestamps'], data['positions_y'], 'g-', label='Y')
        ax_pos.plot(data['timestamps'], data['positions_z'], 'b-', label='Z')
        
        # Add target position lines if available
        if data['target_pose']:
            ax_pos.axhline(y=data['target_pose']['x'], color='r', linestyle='--', alpha=0.5, label='X Target')
            ax_pos.axhline(y=data['target_pose']['y'], color='g', linestyle='--', alpha=0.5, label='Y Target')
            ax_pos.axhline(y=data['target_pose']['z'], color='b', linestyle='--', alpha=0.5, label='Z Target')
        
        ax_pos.set_title('End-Effector Position')
        ax_pos.set_xlabel('Time (s)')
        ax_pos.set_ylabel('Position (m)')
        ax_pos.grid(True)
        ax_pos.legend()
    
    def _plot_velocity_data(self, fig, gs_pos, data):
        """Plot velocity data"""
        ax_vel = fig.add_subplot(gs_pos)
        ax_vel.plot(data['timestamps'], data['velocities_x'], 'r-', label='X')
        ax_vel.plot(data['timestamps'], data['velocities_y'], 'g-', label='Y')
        ax_vel.plot(data['timestamps'], data['velocities_z'], 'b-', label='Z')
        
        ax_vel.set_title('End-Effector Velocity')
        ax_vel.set_xlabel('Time (s)')
        ax_vel.set_ylabel('Velocity (m/s)')
        ax_vel.grid(True)
        ax_vel.legend()
    
    def _plot_position_error(self, fig, gs_pos, data):
        """Plot position error analysis"""
        ax_err = fig.add_subplot(gs_pos)
        
        # Create a bar chart for position errors
        labels = ['X Error', 'Y Error', 'Z Error', 'Euclidean']
        error_values = [
            data['position_error']['x'],
            data['position_error']['y'],
            data['position_error']['z'],
            data['position_error']['euclidean']
        ]
        
        colors = ['red', 'green', 'blue', 'purple']
        bars = ax_err.bar(labels, error_values, color=colors, alpha=0.7)
        
        # Add a horizontal line at zero
        ax_err.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        # Add a horizontal line indicating a 1cm error threshold
        ax_err.axhline(y=0.01, color='r', linestyle='--', alpha=0.5, label='+1cm')
        ax_err.axhline(y=-0.01, color='r', linestyle='--', alpha=0.5, label='-1cm')
        
        # Add text annotations with the error values
        for i, v in enumerate(error_values):
            # Format the text differently for the Euclidean distance (always positive)
            if i == 3:  # Euclidean
                ax_err.annotate(f'{v:.4f}m', 
                              xy=(i, v), 
                              xytext=(0, 5),
                              textcoords='offset points',
                              ha='center', 
                              va='bottom')
            else:
                # For X, Y, Z show the signed error
                ax_err.annotate(f'{v:.4f}m', 
                              xy=(i, v), 
                              xytext=(0, 5 if v >= 0 else -25),
                              textcoords='offset points',
                              ha='center', 
                              va='bottom' if v >= 0 else 'top')
        
        ax_err.set_title('Position Error Analysis')
        ax_err.set_ylabel('Error (m)')
        ax_err.grid(True, axis='y')
        
        # Set y-axis limits to show both positive and negative errors
        max_abs_error = max([abs(v) for v in error_values]) * 1.2  # Add 20% padding
        ax_err.set_ylim(-max_abs_error, max_abs_error)
    
    def _calculate_velocities(self, positions, timestamps, velocities_out):
        """Calculate velocities from position data"""
        if len(positions) < 2:
            if velocities_out:
                velocities_out.clear()
            return
            
        # First point has zero velocity
        velocities_out.append(0.0)
        
        # Calculate velocities using central differences where possible
        for i in range(1, len(timestamps) - 1):
            dt = timestamps[i + 1] - timestamps[i - 1]
            if dt > 0 and not np.isnan(positions[i + 1]) and not np.isnan(positions[i - 1]):
                # Central difference
                vel = (positions[i + 1] - positions[i - 1]) / dt
                velocities_out.append(vel)
            else:
                # Forward difference fallback
                dt = timestamps[i] - timestamps[i - 1]
                if dt > 0 and not np.isnan(positions[i]) and not np.isnan(positions[i - 1]):
                    vel = (positions[i] - positions[i - 1]) / dt
                else:
                    vel = 0.0
                velocities_out.append(vel)
        
        # Last point - backward difference
        i = len(timestamps) - 1
        if i > 0:
            dt = timestamps[i] - timestamps[i - 1]
            if dt > 0 and not np.isnan(positions[i]) and not np.isnan(positions[i - 1]):
                vel = (positions[i] - positions[i - 1]) / dt
            else:
                vel = 0.0
            velocities_out.append(vel)
    
    def _calculate_accelerations(self, velocities, timestamps, accelerations_out):
        """Calculate accelerations from velocity data"""
        if len(velocities) < 2:
            if accelerations_out:
                accelerations_out.clear()
            return
            
        # First point has zero acceleration
        accelerations_out.append(0.0)
        
        # Calculate accelerations using central differences where possible
        for i in range(1, len(timestamps) - 1):
            dt = timestamps[i + 1] - timestamps[i - 1]
            if dt > 0:
                # Central difference
                accel = (velocities[i + 1] - velocities[i - 1]) / dt
                accelerations_out.append(accel)
            else:
                # Forward difference fallback
                dt = timestamps[i] - timestamps[i - 1]
                accel = (velocities[i] - velocities[i - 1]) / dt if dt > 0 else 0.0
                accelerations_out.append(accel)
        
        # Last point - backward difference
        i = len(timestamps) - 1
        if i > 0:
            dt = timestamps[i] - timestamps[i - 1]
            accel = (velocities[i] - velocities[i - 1]) / dt if dt > 0 else 0.0
            accelerations_out.append(accel)
    
    def _interpolate_at_timestamps(self, target_times, source_times, source_values):
        """Interpolate values at specific timestamps"""
        result = []
        
        for time in target_times:
            # Handle timestamps before first velocity measurement
            if time <= source_times[0]:
                result.append(source_values[0])
                continue
                
            # Handle timestamps after last velocity measurement
            if time >= source_times[-1]:
                result.append(source_values[-1])
                continue
            
            # Find surrounding points for interpolation
            for i in range(len(source_times) - 1):
                if source_times[i] <= time <= source_times[i + 1]:
                    # Linear interpolation
                    t = (time - source_times[i]) / (source_times[i + 1] - source_times[i])
                    interpolated = source_values[i] * (1 - t) + source_values[i + 1] * t
                    result.append(interpolated)
                    break
        
        return result
    
    def _get_target_pose(self, target_frame):
        """Get target pose from TF"""
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
            self.get_logger().info(f"Target pose: X={target_pose['x']:.4f}, Y={target_pose['y']:.4f}, Z={target_pose['z']:.4f}")
            return target_pose
        except Exception as e:
            self.get_logger().warn(f'Could not get target pose: {e}')
            return {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            }
    
    def _calculate_position_error(self, final_pos, target_pose):
        """Calculate position error metrics"""
        return {
            'x': final_pos['x'] - target_pose['x'],
            'y': final_pos['y'] - target_pose['y'],
            'z': final_pos['z'] - target_pose['z'],
            'euclidean': np.sqrt(
                (final_pos['x'] - target_pose['x'])**2 +
                (final_pos['y'] - target_pose['y'])**2 +
                (final_pos['z'] - target_pose['z'])**2
            )
        }


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DataRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 