import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    slam_launch_path = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
    ])

    slam_config_path = PathJoinSubstitution([
        FindPackageShare('articubot_one'), 'config', 'slam.yaml'
    ])

    navigation_launch_path = PathJoinSubstitution([
        FindPackageShare('articubot_one'), 'launch', 'navigation.launch.py'
    ])

    nav2_config_path = PathJoinSubstitution([
        FindPackageShare('articubot_one'), 'config', 'navigation.yaml'
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('articubot_one'), 'rviz', 'slam_config.rviz'
    ])

    urdf_path = PathJoinSubstitution([
        FindPackageShare('articubot_one'), 'urdf', 'robot.urdf.xacro'
    ])

    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'humble':
        slam_param_name = 'params_file'

    return LaunchDescription([

        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sim_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='model',
            default_value=urdf_path,
            description='Absolute path to robot urdf file'
        ),

        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        # Launch SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                slam_param_name: slam_config_path
            }.items()
        ),



        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),

        # Optional initial pose publisher
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'articubot_one', 'initial_pose_publisher'],
                    output='screen'
                )
            ]
        )
    ])
