#!/usr/bin/env python
############################################################################
#
# Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
# used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    
    # Launch arguments for collision avoidance configuration
    collision_enabled_arg = DeclareLaunchArgument(
        'collision_enabled',
        default_value='true',
        description='Enable collision avoidance'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='3.0',
        description='Minimum safe distance from obstacles (meters)'
    )
    
    emergency_distance_arg = DeclareLaunchArgument(
        'emergency_distance',
        default_value='1.0',
        description='Emergency stop distance (meters)'
    )
    
    return LaunchDescription([
        # Launch arguments
        collision_enabled_arg,
        min_distance_arg,
        emergency_distance_arg,
        
        # Bridge Gazebo to ROS2
        # LiDAR Bridge
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '--ros-args',
                '-r', '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan'],
            output='screen',
            name='lidar_bridge'
        ),
        
        # TF Publisher Node - IMPORTANT: This publishes the frame tree!
        Node(
            package='px4_offboard',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False  # Set to True if using simulated time
            }]
        ),
        
        # Processes node
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        
        # Control node
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),
        
        # Collision avoidance node (should start before velocity_control)
        Node(
            package='px4_offboard',
            executable='collision_avoidance',
            name='collision_avoidance',
            parameters=[{
                'collision_prevention.enabled': LaunchConfiguration('collision_enabled'),
                'collision_prevention.minimum_distance': LaunchConfiguration('min_distance'),
                'collision_prevention.maximum_distance': 15.0,
                'collision_prevention.slowdown_distance': 5.0,
                'collision_prevention.velocity_reduction_factor': 0.5,
                'collision_prevention.emergency_stop_distance': LaunchConfiguration('emergency_distance'),
                'collision_prevention.sensor_timeout': 2.0,
                'collision_prevention.horizontal_fov': 120.0,
                'collision_prevention.vertical_fov': 30.0,
                'update_rate': 20.0,
            }],
            output='screen'
        ),  

        # Velocity control node (now publishes to collision avoidance)
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity'
        ),
        
        # RViz2 node with TF visualization
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]],
        #     condition=None  # Remove this line to enable RViz
        # )
    ])