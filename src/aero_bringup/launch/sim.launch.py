#!/usr/bin/env python3
"""Launch TurtleBot3 waffle_pi in Gazebo (house world with headless option)."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context)

    tb3_gazebo_share = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')

    # Determine world launch file
    if world == 'house':
        world_launch = os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_house.launch.py')
        if not os.path.exists(world_launch):
            # fallback
            world_launch = os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
    else:
        world_launch = os.path.join(tb3_gazebo_share, 'launch', f'turtlebot3_{world}.launch.py')

    env_vars = []
    if headless == 'true':
        env_vars.append(SetEnvironmentVariable('GAZEBO_GUI', '0'))
        env_vars.append(SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE_SERVER_PLUGIN', 'gz-rendering-ogre2'))

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
    )

    return env_vars + [gazebo_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house',
                              description='World name: house or world'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without GUI (gz sim -s -r)'),
        OpaqueFunction(function=launch_setup),
    ])
