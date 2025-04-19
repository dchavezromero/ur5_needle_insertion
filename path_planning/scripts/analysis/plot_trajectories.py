#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from pathlib import Path

def load_trajectory_data(filename):
    """Load trajectory data from CSV file."""
    df = pd.read_csv(filename)
    print(f"\nColumns in {filename}:")
    print(df.columns.tolist())
    return df

def plot_joint_trajectories(df, title, save_path=None):
    """Plot joint trajectories over time."""
    plt.figure(figsize=(12, 8))
    
    # Joint names and their corresponding column names in the CSV
    joint_mapping = {
        'elbow': ('elbow_joint', 'elbow_joint'),
        'shoulder_lift': ('shoulder_lift_joint', 'shoulder_lift_joint'),
        'shoulder_pan': ('shoulder_pan_joint', 'shoulder_pan_joint'),
        'wrist_1': ('wrist_1_joint', 'wrist_1_joint'),
        'wrist_2': ('wrist_2_joint', 'wrist_2_joint'),
        'wrist_3': ('wrist_3_joint', 'wrist_3_joint')
    }
    
    # Plot each joint
    for joint_name, (planned_col, actual_col) in joint_mapping.items():
        # Plot planned trajectory (solid line)
        plt.plot(df['time'], df[planned_col], 
                label=f'Planned {joint_name}', linewidth=2)
        
        # Plot actual trajectory (dotted line)
        plt.plot(df['time'], df[actual_col], 
                label=f'Actual {joint_name}', linestyle='--', linewidth=2)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    plt.title(title)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)
    plt.tight_layout()
    
    if save_path:
        # Ensure the directory exists
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"Saved plot to: {save_path}")
    plt.close()  # Close the figure to free memory

def calculate_metrics(df):
    """Calculate various metrics for the trajectory."""
    metrics = {}
    
    # Time duration
    metrics['duration'] = df['time'].max() - df['time'].min()
    
    # Joint names and their corresponding column names
    joint_mapping = {
        'elbow': ('elbow_joint', 'elbow_joint'),
        'shoulder_lift': ('shoulder_lift_joint', 'shoulder_lift_joint'),
        'shoulder_pan': ('shoulder_pan_joint', 'shoulder_pan_joint'),
        'wrist_1': ('wrist_1_joint', 'wrist_1_joint'),
        'wrist_2': ('wrist_2_joint', 'wrist_2_joint'),
        'wrist_3': ('wrist_3_joint', 'wrist_3_joint')
    }
    
    # Maximum velocity (approximate) for both planned and actual
    max_velocities = {'planned': {}, 'actual': {}}
    for joint_name, (planned_col, actual_col) in joint_mapping.items():
        # Planned velocities
        planned_velocities = np.diff(df[planned_col]) / np.diff(df['time'])
        max_velocities['planned'][joint_name] = np.max(np.abs(planned_velocities))
        
        # Actual velocities
        actual_velocities = np.diff(df[actual_col]) / np.diff(df['time'])
        max_velocities['actual'][joint_name] = np.max(np.abs(actual_velocities))
    
    metrics['max_velocities'] = max_velocities
    
    # Path length for both planned and actual
    path_lengths = {'planned': 0, 'actual': 0}
    for i in range(1, len(df)):
        planned_diff = 0
        actual_diff = 0
        for joint_name, (planned_col, actual_col) in joint_mapping.items():
            planned_diff += (df[planned_col].iloc[i] - df[planned_col].iloc[i-1])**2
            actual_diff += (df[actual_col].iloc[i] - df[actual_col].iloc[i-1])**2
        path_lengths['planned'] += np.sqrt(planned_diff)
        path_lengths['actual'] += np.sqrt(actual_diff)
    
    metrics['path_lengths'] = path_lengths
    
    # Calculate convergence time (time until actual position is within threshold of planned)
    convergence_threshold = 0.01  # rad
    convergence_times = {}
    for joint_name, (planned_col, actual_col) in joint_mapping.items():
        planned_final = df[planned_col].iloc[-1]
        actual_positions = df[actual_col]
        convergence_idx = np.where(np.abs(actual_positions - planned_final) < convergence_threshold)[0]
        if len(convergence_idx) > 0:
            convergence_times[joint_name] = df['time'].iloc[convergence_idx[0]]
        else:
            convergence_times[joint_name] = df['time'].iloc[-1]
    
    metrics['convergence_times'] = convergence_times
    metrics['max_convergence_time'] = max(convergence_times.values())
    
    return metrics

def main():
    # Get the directory where this script is located
    script_dir = Path(__file__).parent.absolute()
    # Go up one directory to the cpp directory
    cpp_dir = script_dir.parent / 'cpp'
    
    print(f"Looking for trajectory files in: {cpp_dir}")
    
    # Find all trajectory CSV files
    csv_files = list(cpp_dir.glob('trajectory_*.csv'))
    
    if not csv_files:
        print(f"No trajectory files found in {cpp_dir}!")
        print("Make sure you've run the C++ node and it's saving files in the correct location.")
        return
    
    print(f"Found {len(csv_files)} trajectory files:")
    for f in csv_files:
        print(f"  - {f.name}")
    
    # Create output directory for plots
    output_dir = script_dir / 'trajectory_analysis'
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nSaving plots to: {output_dir}")
    
    # Analyze each trajectory
    for csv_file in csv_files:
        print(f"\nAnalyzing {csv_file.name}...")
        
        try:
            # Load data
            df = load_trajectory_data(csv_file)
            
            # Calculate metrics
            metrics = calculate_metrics(df)
            
            # Print metrics
            print("\nMetrics:")
            print(f"Duration: {metrics['duration']:.2f} seconds")
            print("\nPath Lengths:")
            print(f"Planned: {metrics['path_lengths']['planned']:.2f} rad")
            print(f"Actual: {metrics['path_lengths']['actual']:.2f} rad")
            
            print("\nMaximum Velocities:")
            print("Planned:")
            for joint, vel in metrics['max_velocities']['planned'].items():
                print(f"  {joint}: {vel:.2f} rad/s")
            print("Actual:")
            for joint, vel in metrics['max_velocities']['actual'].items():
                print(f"  {joint}: {vel:.2f} rad/s")
            
            print("\nConvergence Times (time until within 0.01 rad of final position):")
            for joint, time in metrics['convergence_times'].items():
                print(f"  {joint}: {time:.2f} seconds")
            print(f"Maximum convergence time: {metrics['max_convergence_time']:.2f} seconds")
            
            # Plot trajectories
            plot_title = f"Joint Trajectories - {csv_file.stem}"
            plot_path = output_dir / f"{csv_file.stem}.png"
            plot_joint_trajectories(df, plot_title, plot_path)
            
        except Exception as e:
            print(f"Error processing {csv_file.name}: {str(e)}")

if __name__ == "__main__":
    main() 