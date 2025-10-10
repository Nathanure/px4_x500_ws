#!/usr/bin/env python3
"""
Static Transform Publisher for PX4 Navigation
Publishes static transforms for sensor frames
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch argument for sim time
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sim_time'
        ),

        # Static transform: base_link -> link (lidar frame)
        # Adjust the translation values based on your actual lidar mounting position
        # This assumes the lidar is mounted at the center of the drone
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '0', '0', '0',  # x, y, z translation
                '0', '0', '0',  # roll, pitch, yaw rotation
                'base_link',    # parent frame
                'link'          # child frame (lidar frame name)
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim')
            }]
        ),
    ])