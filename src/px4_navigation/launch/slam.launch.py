#!/usr/bin/env python3
# PX4 SLAM Launch File
# Adapted for X500 quadcopter with 2D lidar

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
    
    static_tf_launch = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'launch', 'static_transforms.launch.py']
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
            default_value='false',
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

        # ============ Static Transforms ============
        # Publishes base_link -> link (lidar frame)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch),
            launch_arguments={
                'sim': LaunchConfiguration('sim')
            }.items()
        ),

        # ============ TF Publisher Node ============
        # This publishes odom->base_link and publishes /odom topic
        Node(
            package='px4_navigation',
            executable='tf_publisher',
            name='px4_tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim')
            }]
        ),

        # ============ SLAM Toolbox ============
        # This publishes map->odom transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('sim'),
                slam_param_name: slam_config_path,
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw')
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