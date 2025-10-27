#!/usr/bin/env python3
# PX4 Navigation Launch File
# Adapted for X500 quadcopter with Nav2

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Paths
    map_path = 'lidar_2d_walls'  # Specific map file for navigation

    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    nav2_launch_path = PathJoinSubstitution(
        [nav2_bringup_dir, 'launch', 'navigation_launch.py']
    )

    nav_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'config', 'navigation.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('px4_navigation'), 'rviz', 'px4_navigation.rviz']
    )

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            name='sim',
            default_value='True',
            description='Enable use_sim_time to true'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('px4_navigation'), 'maps', f'{map_path}.yaml']
            ),
            description='Full path to map yaml file'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='True',
            description='Run RViz2'
        ),

        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Top-level namespace'
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

        # ============ Map Server ============
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('sim')},
                {'yaml_filename': map_file}
            ]
        ),

        # ============ AMCL (Localization) ============
        # THIS IS THE KEY FIX - Pass initial pose parameters to AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav_config_path,
                {
                    'use_sim_time': LaunchConfiguration('sim'),
                    'set_initial_pose': True,
                    'initial_pose.x': LaunchConfiguration('initial_pose_x'),
                    'initial_pose.y': LaunchConfiguration('initial_pose_y'),
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
                }
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),

        # ============ Lifecycle Manager for Map Server ============
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('sim')},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),

        # ============ Nav2 Stack ============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('sim'),
                'params_file': nav_config_path,
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items()
        ),

        # ============ cmd_vel Bridge ============
        # Converts Nav2 /cmd_vel to PX4 /offboard_velocity_cmd
        Node(
            package='px4_navigation',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim'),
                'enable_bridge': True,
            }]
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