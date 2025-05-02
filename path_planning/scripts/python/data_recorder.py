#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import csv
import threading
import datetime
import os
from builtin_interfaces.msg import Time


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
        
        # Get use_sim_time parameter
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'Data Recorder Node Initialized with use_sim_time={use_sim_time}')
        
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
            Trigger,
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
            
        self.get_logger().info('Started recording joint states')
        response.success = True
        response.message = 'Recording started'
        return response
    
    def stop_recording_callback(self, request, response):
        states_copy = []
        
        with self.recording_lock:
            self.recording = False
            states_copy = list(self.recorded_states)
        
        # Save the recorded data
        if states_copy:
            self.save_joint_states_to_csv(states_copy)
            self.get_logger().info(f'Stopped recording and saved {len(states_copy)} joint states')
            response.success = True
            response.message = 'Recording stopped and data saved'
        else:
            self.get_logger().warn('No joint states recorded, no CSV file generated')
            response.success = True
            response.message = 'Recording stopped but no data was recorded'
        
        return response
    
    def get_current_timestamp(self):
        now = datetime.datetime.now()
        return now.strftime("%Y%m%d_%H%M%S")
    
    def save_joint_states_to_csv(self, joint_states):
        if not joint_states:
            self.get_logger().warn('No joint states recorded, skipping CSV generation')
            return
        
        timestamp = self.get_current_timestamp()
        filename = f'joint_trajectory_{timestamp}.csv'
        
        # List of joints we're interested in
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        try:
            with open(filename, 'w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                
                # Write header
                header = ['time'] + joint_names
                csv_writer.writerow(header)
                
                # Get indices of the joints we're interested in for the first message
                joint_indices = []
                first_state = joint_states[0]
                
                for joint_name in joint_names:
                    try:
                        joint_indices.append(first_state.name.index(joint_name))
                    except ValueError:
                        self.get_logger().error(f'Joint {joint_name} not found in recorded joint states')
                        return
                
                # Get the start time from the first message
                first_msg = joint_states[0]
                start_sec = first_msg.header.stamp.sec
                start_nsec = first_msg.header.stamp.nanosec
                
                # Write joint states
                for state in joint_states:
                    # Calculate time relative to the first message in seconds
                    time_secs = (state.header.stamp.sec - start_sec) + ((state.header.stamp.nanosec - start_nsec) * 1e-9)
                    
                    # Handle negative time that might occur due to precision errors
                    if time_secs < 0:
                        time_secs = 0.0
                    
                    row = [time_secs]
                    
                    # Write joint positions
                    for idx in joint_indices:
                        if idx < len(state.position):
                            row.append(state.position[idx])
                        else:
                            self.get_logger().warn(f'Missing position data for joint at index {idx}')
                            row.append('NaN')  # Handle missing data
                    
                    csv_writer.writerow(row)
            
            self.get_logger().info(f'Saved {len(joint_states)} joint states to {filename}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to save joint states to CSV: {str(e)}')


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