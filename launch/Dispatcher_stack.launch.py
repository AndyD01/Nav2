#!/usr/bin/env python3
"""
Dispatcher_stack.launch.py
Porneste station_manager, dispatcher, mission_executor intr-un singur launch.

Folosire:
  ros2 launch amr2ax_nav2 Dispatcher_stack.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr2ax_nav2',
            executable='station_manager.py',
            name='station_manager',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='amr2ax_nav2',
            executable='dispatcher.py',
            name='dispatcher',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='amr2ax_nav2',
            executable='mission_executor_v2.py',
            name='mission_executor',
            output='screen',
            emulate_tty=True,
            arguments=['--topic'],
        ),
    ])
