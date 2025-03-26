#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
    rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', '/home/altair/gaitame/rviz/gaitame_config.rviz'],
        output='screen'
    )
    return LaunchDescription([serial_node, rviz_node, rviz_process])
