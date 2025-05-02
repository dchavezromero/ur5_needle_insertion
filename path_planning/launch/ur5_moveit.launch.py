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
    # Paths and constants
    world_file = os.path.join(get_package_share_directory('path_planning'), 'worlds', 'empty.world')
    hospital_models_path = get_package_share_directory("hospital_models")
    
    # Initialize MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur5")
        .robot_description(file_path="config/ur5.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Apply simulation time setting
    use_sim_time = {"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    
    # === Simulation Environment ===
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items(),
    )
    
    # === Visualization ===
    # RViz configuration
    rviz_base = os.path.join(get_package_share_directory("ur5_moveit_config"), "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    # === Robot Control Components ===
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
    )
    
    # Controllers
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
    
    # === Hospital Environment Objects ===
    needle_gap = 0.10
    # 1. Bedside Table
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(hospital_models_path, "models/BedsideTable/model.sdf"),
            '-entity', 'bedsideTable',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    
    table_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_broadcaster",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "bedsideTable"]
    )
    
    # 2. Patient
    spawn_patient = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(hospital_models_path, "models/ElderMalePatient/model.sdf"),
            '-entity', 'elderMalePatient',
            '-x', '0.94', '-y', '0', '-z', '0',
            '-Y', '-1.57'  # Apply a -1.57 radian yaw rotation
        ],
        output='screen'
    )
    
    patient_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="patient_broadcaster",
        output="log",
        arguments=["0.94", "0", "0", "0", "0", "-0.7071068", "0.7071068", "world", "elderMalePatient"]
    )
    
    # 3. Side Table
    spawn_side_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(hospital_models_path, "models/BedTable/model.sdf"),
            '-entity', 'bedTable',
            '-x', '0.855', '-y', '-1.383', '-z', '0'
        ],
        output='screen'
    )
    
    side_table_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="side_table_broadcaster",
        output="log",
        arguments=["0.855", "-1.383", "0", "0", "0", "0", "world", "bedTable"]
    )
    
    # 4. Surgeon
    spawn_surgeon = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(hospital_models_path, "models/OpScrubs/model.sdf"),
            '-entity', 'opScrubs',
            '-x', '0.127641', '-y', '-0.716872', '-z', '0'
        ],
        output='screen'
    )
    
    surgeon_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="surgeon_broadcaster",
        output="log",
        arguments=["0.127641", "-0.716872", "0", "0", "0", "0", "world", "opScrubs"]
    )
    
    # 5. Divider
    divider_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="divider_tf",
        output="log",
        arguments=["0.94", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "divider"]
    )
    
    # === Medical Insertion Points ===
    torso_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="torso_insertion_broadcaster",
        output="log",
        arguments=[str(-0.3), str(-0.1), str(1.0 + needle_gap), str(0.0), str(3.14), str(0.0), "elderMalePatient", "torso_insertion_point"]
    )
    # Torso insertion point
    torso2_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="torso2_insertion_broadcaster",
        output="log",
        arguments=[str(-0.15), str(-0.1), str(0.975 + needle_gap), str(0.0), str(3.14), str(0.0), "elderMalePatient", "torso7_insertion_point"]
    )
    
    # Arm insertion point
    arm_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm_insertion_broadcaster",
        output="log",
        arguments=[str(-0.4), str(-0.35 - needle_gap), str(0.85), str(1.57), str(1.57), str(0.0), "elderMalePatient", "arm_insertion_point"]
    )

    arm2_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm2_insertion_broadcaster",
        output="log",
        arguments=[str(-0.365), str(-0.25 - needle_gap), str(0.975), str(1.57), str(1.57), str(0.0), "elderMalePatient", "arm2_insertion_point"]
    )

    arm3_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm3_insertion_broadcaster",
        output="log",
        arguments=[str(-0.50), str(-0.275 - needle_gap), str(0.9), str(1.57), str(1.57), str(0.0), "elderMalePatient", "arm3_insertion_point"]
    )

    # Leg insertion point
    leg_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leg_insertion_broadcaster",
        output="log",
        arguments=[str(0.25), str(-0.125), str(0.9 + needle_gap), str(0), str(3.14), str(0), "elderMalePatient", "leg_insertion_point"]
    )

    leg2_insertion_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leg2_insertion_broadcaster",
        output="log",
        arguments=[str(0.25), str(-0.2 - needle_gap), str(0.85), str(1.57), str(1.57), str(0), "elderMalePatient", "leg2_insertion_point"]
    )
    
    # === Robot ===
    # Spawn the UR5 on top of the table
    spawn_ur5 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'ur5',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    
    # === Collision Detection ===
    # Collision publisher for planning scene
    collision_publisher = Node(
        package='path_planning',
        executable='collision_publisher.py',
        name='collision_publisher',
        output='screen'
    )

    data_recorder = Node(
        package='path_planning',
        executable='data_recorder.py',
        name='data_recorder',
        output='screen'
    )

    motion_planning_service = Node(
        package='path_planning',
        executable='motion_planning_service',
        name='motion_planning_service',
        output='screen'
    )
    
    # Create launch description with all the nodes
    return LaunchDescription([
        # Simulation
        gazebo,
        
        # Robot
        robot_state_publisher,
        spawn_ur5,
        
        # Hospital Environment
        spawn_table,
        table_static_tf,
        spawn_patient,
        patient_static_tf,
        spawn_surgeon,
        surgeon_static_tf,
        spawn_side_table,
        side_table_static_tf,
        divider_tf,
        
        # Insertion Points
        torso_insertion_tf,
        torso2_insertion_tf,
        arm_insertion_tf,
        arm2_insertion_tf,
        arm3_insertion_tf,
        leg_insertion_tf,
        leg2_insertion_tf,
        # Planning & Control
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        
        # Visualization
        rviz_node,
        
        # Collision Detection
        collision_publisher,
        #data_recorder,
        #motion_planning_service
    ])