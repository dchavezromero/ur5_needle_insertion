import os
import launch
import launch_ros.actions
import launch.substitutions

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ur5_robot_description').find('ur5_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur5.xacro')

    return launch.LaunchDescription([
        # Joint State Publisher
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),


        # Robot State Publisher (Ensuring correct string parsing)
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.Command(['xacro ', xacro_file]),
                    value_type=str
                )
            }]
        ),

        # RViz (for visualization)
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'config.rviz')],
            output='screen'
        )
    ])
