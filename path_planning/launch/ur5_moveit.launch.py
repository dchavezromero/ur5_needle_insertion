import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('path_planning'), 'worlds', 'empty.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world_file}.items(),
    )
    moveit_config = (
        MoveItConfigsBuilder("ur5")
        .robot_description(file_path="config/ur5.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"]) # maybe change to RRTStar
        .to_moveit_configs()
    )
    # Start the actual move_group node/action server
    use_sim_time = {"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
    )
    # RViz
    rviz_base = os.path.join(os.path.join(get_package_share_directory("ur5_moveit_config"), "config"))
    rviz_empty_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )   
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    # )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
    get_package_share_directory("ur5_robot_description"),
    "config",
    "ur5_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_arm_controller", "-c", "/controller_manager"],
    )

    # hand_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    # )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
             output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ur5_arm_controller'],
            output='screen'
    )
    
    # Spawn the Table in Gazebo
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory("hospital_models"), "models/BedsideTable/model.sdf"),
            '-entity', 'bedsideTable',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Add static transform for table
    table_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_broadcaster",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "bedsideTable"]
    )

    spawn_patient = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory("hospital_models"), "models/ElderMalePatient/model.sdf"),
            '-entity', 'elderMalePatient',
            '-x', '0.94', '-y', '0', '-z', '0',
            '-Y', '-1.57'  # Apply a -1.57 radian yaw rotation
        ],
        output='screen'
    )

    # Add static transform for patient position
    patient_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="patient_broadcaster",
        output="log",
        arguments=["0.94", "0", "0", "0", "0", "-0.7071068", "0.7071068", "world", "elderMalePatient"]
    )
    
    torso7_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="torso7_insertion_broadcaster",
        output="log",
        arguments=["-0.15", "-0.1", "0.975", "0", "0", "0", "elderMalePatient", "torso7_insertion_point"]
    )

    # Add static transform for the arm_insertion_point relative to patient
    arm_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm_insertion_broadcaster",
        output="log",
    arguments=["-0.4", "-0.35", "0.85", "1.57", "0", "0", "elderMalePatient", "arm_insertion_point"]
    )
    
    # Add static transform for the leg_insertion_point relative to patient
    leg_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leg_insertion_broadcaster",
        output="log",
        arguments=["0.45", "0.05", "1.0", "0", "0", "0", "elderMalePatient", "leg_insertion_point"]
    )

    spawn_side_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory("hospital_models"), "models/BedTable/model.sdf"),
            '-entity', 'bedTable',
            '-x', '0.855', '-y', '-1.383', '-z', '0'
        ],
        output='screen'
    )

    spawn_surgeon = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory("hospital_models"), "models/OpScrubs/model.sdf"),
            '-entity', 'opScrubs',
            '-x', '0.127641', '-y', '-0.716872', '-z', '0'
        ],
        output='screen'
    )

    # Spawn the UR5 on top of the table
    spawn_ur5 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'ur5',
            '-x', '0', '-y', '0', '-z', '0'  # Adjust based on table height
        ],
        output='screen'
    )
    

    return LaunchDescription([
            gazebo,
            robot_state_publisher,
            # static_tf,
            spawn_table,
            spawn_ur5,
            spawn_patient,
            # spawn_surgeon,
            # spawn_side_table,
            rviz_node,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            patient_static_tf,
            leg_insertion_tf,
            arm_insertion_tf,
            torso7_insertion_tf,
            table_static_tf
        ])

    # return LaunchDescription([
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_entity,
    #             on_exit=[load_joint_state_controller],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_controller,
    #             on_exit=[load_arm_controller],
    #         )
    #     ),
    #     gazebo,
    #     node_robot_state_publisher,
    #     spawn_entity
    # ])  # Ensure it returns a valid launch description