from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_qwiic_icm_20948',
            executable='ros_qwiic_icm_20948',
            name='ros_qwiic_icm_20948',
            output='screen'),
    ])