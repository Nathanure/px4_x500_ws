#!/usr/bin/env python3
# PX4 Navigation Launch File
# Adapted for X500 quadcopter with 2D lidar

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


MAP_NAME = 'my_map'  # Change to your map name

def generate_launch_description():
    # Paths
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'rviz', 'px4_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'config', 'navigation.yaml']
    )
    
    static_tf_launch = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'launch', 'static_transforms.launch.py']
    )

    return LaunchDescription([
        # ============ Launch Arguments ============
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
            name='map',
            default_value=default_map_path,
            description='Full path to map YAML file'
        ),

        DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.0',
            description='Initial robot X position in map frame'
        ),

        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0',
            description='Initial robot Y position in map frame'
        ),

        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='0.0',
            description='Initial robot yaw in map frame'
        ),

        DeclareLaunchArgument(
            name='autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            name='enable_bridge',
            default_value='true',
            description='Enable CMD_VEL bridge to px4_offboard'
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
        # Publishes odom->base_link and /odom topic from PX4 data
        Node(
            package='px4_navigation',
            executable='tf_publisher',
            name='px4_tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim')
            }]
        ),

        # ============ CMD_VEL Bridge Node ============
        # Bridges Nav2's /cmd_vel to px4_offboard's /offboard_velocity_cmd
        Node(
            package='px4_navigation',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'velocity_scale': 1.0,
                'max_linear_vel': 1.0,
                'max_angular_vel': 1.0
            }],
            condition=IfCondition(LaunchConfiguration('enable_bridge'))
        ),

        # ============ Nav2 Stack ============
        # Includes AMCL, planners, controllers, behavior servers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('sim'),
                'params_file': nav2_config_path,
                'autostart': LaunchConfiguration('autostart'),
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