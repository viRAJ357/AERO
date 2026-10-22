#!/usr/bin/env python3
"""Full AERO stack: Gazebo + SLAM + Nav2 + Perception + Explorer + Commander + Dashboard."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='house')
    headless = LaunchConfiguration('headless', default='false')
    use_llm = LaunchConfiguration('use_llm', default='true')
    load_previous = LaunchConfiguration('load_previous', default='false')

    pkg_bringup = FindPackageShare('aero_bringup')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_bringup, '/launch/sim.launch.py']),
        launch_arguments={'world': world, 'headless': headless}.items(),
    )

    nav_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_bringup, '/launch/nav_slam.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_bringup, '/launch/exploration.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    perception_nodes = [
        Node(
            package='aero_perception',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[
                PathJoinSubstitution([pkg_bringup, 'config', 'yolo_params.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='aero_perception',
            executable='object_localizer',
            name='object_localizer',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='aero_perception',
            executable='semantic_map',
            name='semantic_map',
            parameters=[{
                'use_sim_time': use_sim_time,
                'load_previous': load_previous
            }],
            output='screen',
        ),
    ]

    safety_node = Node(
        package='aero_safety',
        executable='safety_gate',
        name='safety_gate',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    commander_node = Node(
        package='aero_commander',
        executable='llm_commander',
        name='llm_commander',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_llm': use_llm
        }],
        output='screen',
    )

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_bringup, '/launch/dashboard.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    metrics_node = Node(
        package='aero_eval',
        executable='metrics_logger',
        name='metrics_logger',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='house'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_llm', default_value='true'),
        DeclareLaunchArgument('load_previous', default_value='false'),
        sim,
        nav_slam,
        exploration,
        *perception_nodes,
        safety_node,
        commander_node,
        dashboard,
        metrics_node,
    ])
