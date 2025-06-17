from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'view_rplidar_c1_launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "config", "ekf.yaml"]
    )

    extra_launch_path = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'extra.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='extra', 
            default_value='False',
            description='Launch extra launch file'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            condition=IfCondition(PythonExpression(["not ", LaunchConfiguration("extra")]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(extra_launch_path),
            condition=IfCondition(LaunchConfiguration("extra")),
        )
    ])
