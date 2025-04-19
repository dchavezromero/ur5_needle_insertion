#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from pathlib import Path

def load_trajectory_data(filename):
    """load trajectory data from csv file."""
    df = pd.read_csv(filename)
    return df

def plot_joint_trajectories(df, title, save_path=None):
    """plot joint trajectories over time."""
    plt.figure(figsize=(12, 8))
    
    # joint names without the planned/actual prefix
    joint_names = ['elbow', 'shoulder_lift', 'shoulder_pan',
                  'wrist_1', 'wrist_2', 'wrist_3']
    
    # plot each joint
    for joint in joint_names:
        # plot planned trajectory (solid line)
        plt.plot(df['time'], df[f'planned_{joint}'], 
                label=f'Planned {joint}', linewidth=2)
        
        # plot actual trajectory (dotted line)
        plt.plot(df['time'], df[f'actual_{joint}'], 
                label=f'Actual {joint}', linestyle='--', linewidth=2)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    plt.title(title)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.show()

def calculate_metrics(df):
    """calculate various metrics for the trajectory."""
    metrics = {}
    
    # time duration
    metrics['duration'] = df['time'].max() - df['time'].min()
    
    # maximum velocity (approximate) for both planned and actual
    joint_names = ['elbow', 'shoulder_lift', 'shoulder_pan',
                  'wrist_1', 'wrist_2', 'wrist_3']
    
    max_velocities = {'planned': {}, 'actual': {}}
    for joint in joint_names:
        # planned velocities
        planned_velocities = np.diff(df[f'planned_{joint}']) / np.diff(df['time'])
        max_velocities['planned'][joint] = np.max(np.abs(planned_velocities))
        
        # actual velocities
        actual_velocities = np.diff(df[f'actual_{joint}']) / np.diff(df['time'])
        max_velocities['actual'][joint] = np.max(np.abs(actual_velocities))
    
    metrics['max_velocities'] = max_velocities
    
    # path length for both planned and actual
    path_lengths = {'planned': 0, 'actual': 0}
    for i in range(1, len(df)):
        planned_diff = 0
        actual_diff = 0
        for joint in joint_names:
            planned_diff += (df[f'planned_{joint}'].iloc[i] - df[f'planned_{joint}'].iloc[i-1])**2
            actual_diff += (df[f'actual_{joint}'].iloc[i] - df[f'actual_{joint}'].iloc[i-1])**2
        path_lengths['planned'] += np.sqrt(planned_diff)
        path_lengths['actual'] += np.sqrt(actual_diff)
    
    metrics['path_lengths'] = path_lengths
    
    return metrics

def main():
    # get the directory where this script is located
    script_dir = Path(__file__).parent.absolute()
    # go up one directory to the cpp directory
    cpp_dir = script_dir.parent / 'cpp'
    
    print(f"looking for trajectory files in: {cpp_dir}")
    
    # find all trajectory csv files
    csv_files = list(cpp_dir.glob('trajectory_*.csv'))
    
    if not csv_files:
        print(f"no trajectory files found in {cpp_dir}!")
        print("make sure you've run the c++ node and it's saving files in the correct location.")
        return
    
    print(f"found {len(csv_files)} trajectory files:")
    for f in csv_files:
        print(f"  - {f.name}")
    
    # create output directory for plots
    output_dir = script_dir / 'trajectory_analysis'
    output_dir.mkdir(exist_ok=True)
    
    # analyze each trajectory
    for csv_file in csv_files:
        print(f"\nanalyzing {csv_file.name}...")
        
        try:
            # load data
            df = load_trajectory_data(csv_file)
            
            # calculate metrics
            metrics = calculate_metrics(df)
            
            # print metrics
            print("\nmetrics:")
            print(f"duration: {metrics['duration']:.2f} seconds")
            print("\npath lengths:")
            print(f"planned: {metrics['path_lengths']['planned']:.2f} rad")
            print(f"actual: {metrics['path_lengths']['actual']:.2f} rad")
            
            print("\nmaximum velocities:")
            print("planned:")
            for joint, vel in metrics['max_velocities']['planned'].items():
                print(f"  {joint}: {vel:.2f} rad/s")
            print("actual:")
            for joint, vel in metrics['max_velocities']['actual'].items():
                print(f"  {joint}: {vel:.2f} rad/s")
            
            # plot trajectories
            plot_title = f"joint trajectories - {csv_file.stem}"
            plot_path = output_dir / f"{csv_file.stem}.png"
            plot_joint_trajectories(df, plot_title, plot_path)
            
        except Exception as e:
            print(f"error processing {csv_file.name}: {str(e)}")

if __name__ == "__main__":
    main()