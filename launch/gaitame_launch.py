#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_node = Node(
        package='gaitamepkg',
        executable='serialnode',
        name='serial_node'
    )
    rviz_node = Node(
        package='gaitamepkg',
        executable='rviznode',
        name='rviz_node'
    )
    return LaunchDescription([serial_node, rviz_node])
