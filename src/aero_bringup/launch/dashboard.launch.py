#!/usr/bin/env python3
"""Launch the AERO web dashboard."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('port', default_value='8000'),
        Node(
            package='aero_dashboard',
            executable='dashboard_server',
            name='dashboard',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'port': LaunchConfiguration('port')},
            ],
            output='screen',
        ),
    ])
