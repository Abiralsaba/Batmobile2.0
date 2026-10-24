"""Perception only. Camera ownership is opt-in; never launches the ZED wrapper."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_bridge', default_value='false',
                              description='Start the sole direct-SDK camera owner'),
        Node(package='mars_rover', executable='zed_body_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('start_bridge'))),
        Node(package='mars_rover', executable='vision', output='screen'),
    ])
