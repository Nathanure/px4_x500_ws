#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def generate_launch_description():
    # CRITICAL: Kill any existing bridge processes
    subprocess.run(['pkill', '-9', '-f', 'parameter_bridge'], 
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    
    package_dir = get_package_share_directory('px4_offboard')
    config_file = os.path.join(package_dir, 'config', 'collision_prevention_params.yaml')
    
    # CONFIRMED working Gazebo topic from gz topic -l
    gz_lidar_topic = '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan'
    
    return LaunchDescription([
        # FIXED: Use direct command-line arguments (NOT config file for bridge)
        # Single bridge for both clock and lidar - avoids race conditions
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'{gz_lidar_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '--ros-args',
                '-r', f'{gz_lidar_topic}:=/scan'
            ],
            output='screen',
            name='gz_bridge',
            respawn=False
        ),
        
        # Launch PX4 processes (MicroXRCE, SITL, QGC)
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),

        # TF Publisher Node - publishes the frame tree
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='scan_frame_fix',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'link'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='px4_offboard',
            # namespace='px4_offboard',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_footprint'
            }]
        ),
        
        # Bridge: ROS2 → PX4 (lidar_to_obstacle_distance)
        # Wait for /scan topic to be available
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='Starting lidar_bridge node...'),
                Node(
                    package='px4_offboard',
                    namespace='px4_offboard',
                    executable='lidar_bridge',
                    name='lidar_bridge',
                    parameters=[
                        config_file,
                        {
                            'lidar_topic': '/scan',
                            'use_sim_time': True
                        }
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Collision Prevention (reads PX4 obstacle data)
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='px4_offboard',
                    namespace='px4_offboard',
                    executable='collision_prevention',
                    name='collision_prevention',
                    parameters=[
                        config_file,
                        {'use_sim_time': True}
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Manual Control Input
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),
        
        # Velocity Controller (sends to PX4)
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # # RViz Visualization
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])