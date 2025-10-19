#!/usr/bin/env python3
"""
PX4 SLAM Launch File with Nav2 Integration
FIXED: Only launches odom publisher (TF handled by px4_offboard)
"""

import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'config', 'slam.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'rviz', 'px4_slam.rviz']
    )

    # Check ROS distro for parameter naming compatibility
    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'foxy': 
        slam_param_name = 'params_file'

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            name='sim',
            default_value='true',
            description='Enable use_sim_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run RViz2'
        ),

        DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.0',
            description='Initial robot X position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0',
            description='Initial robot Y position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='0.0',
            description='Initial robot yaw'
        ),

        # ============ SLAM Toolbox ============
        # Subscribes to /scan and /odom
        # Publishes map->odom transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('sim'),
                slam_param_name: slam_config_path,
            }.items()
        ),

        # ============ RViz2 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('sim')}]
        )
    ])