import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.substitutions

from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
    )
    
    package_path = os.path.join(
        get_package_share_directory('ur5_robot_description'))

    xacro_file = os.path.join(package_path,
                              'urdf',
                              'ur5.xacro')
    
    # doc = xacro.parse(open(xacro_file))
    # params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
                'robot_description': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.Command(['xacro ', xacro_file]),
                    value_type=str
                )
            }]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
             output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ur5_arm_controller'],
            output='screen'
    )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'ur5'],
                        output='screen')
                        

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])  # Ensure it returns a valid launch description
