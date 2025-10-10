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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.clock import Clock
import numpy as np
from math import cos, sin, atan2, sqrt, pi
import time

from px4_msgs.msg import (
    ObstacleDistance,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
    TrajectorySetpoint,
    OffboardControlMode
)
from geometry_msgs.msg import TwistStamped, PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

class CollisionPrevention(Node):
    def __init__(self):
        super().__init__('collision_prevention')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('collision_prevention.enabled', True),
                ('collision_prevention.minimum_distance', 3.0),  # meters
                ('collision_prevention.maximum_distance', 15.0),  # meters
                ('collision_prevention.slowdown_distance', 5.0),  # meters
                ('collision_prevention.velocity_reduction_factor', 0.5),
                ('collision_prevention.emergency_stop_distance', 1.0),  # meters
                ('collision_prevention.sensor_timeout', 2.0),  # seconds
                ('collision_prevention.horizontal_fov', 120.0),  # degrees
                ('collision_prevention.vertical_fov', 30.0),  # degrees
                ('update_rate', 20.0),  # Hz
            ]
        )
        
        # Get parameters
        self.enabled = self.get_parameter('collision_prevention.enabled').value
        self.min_distance = self.get_parameter('collision_prevention.minimum_distance').value
        self.max_distance = self.get_parameter('collision_prevention.maximum_distance').value
        self.slowdown_distance = self.get_parameter('collision_prevention.slowdown_distance').value
        self.velocity_reduction = self.get_parameter('collision_prevention.velocity_reduction_factor').value
        self.emergency_distance = self.get_parameter('collision_prevention.emergency_stop_distance').value
        self.sensor_timeout = self.get_parameter('collision_prevention.sensor_timeout').value
        self.h_fov = np.deg2rad(self.get_parameter('collision_prevention.horizontal_fov').value)
        self.v_fov = np.deg2rad(self.get_parameter('collision_prevention.vertical_fov').value)
        self.update_rate = self.get_parameter('update_rate').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State variables
        self.vehicle_attitude = None
        self.vehicle_position = None
        self.vehicle_status = None
        self.last_sensor_time = 0
        self.obstacles = {}  # Dictionary to store obstacle data
        self.sector_distances = np.ones(72) * self.max_distance  # 72 sectors of 5 degrees each
        self.offboard_mode = False
        
        # Obstacle distance message setup
        self.obstacle_msg = ObstacleDistance()
        self.obstacle_msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        self.obstacle_msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        self.obstacle_msg.increment = 5.0  # 5 degree increments
        self.obstacle_msg.min_distance = int(self.min_distance * 100)  # convert to cm
        self.obstacle_msg.max_distance = int(self.max_distance * 100)  # convert to cm
        self.obstacle_msg.angle_offset = 0.0
        
        # Subscribers
        self.velocity_cmd_sub = self.create_subscription(
            TwistStamped,
            '/offboard/velocity_setpoint',
            self.velocity_cmd_callback,
            qos_profile
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_profile
        )
        
        # Sensor subscribers (multiple sensor support)
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/obstacle_pointcloud',
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.obstacle_distance_pub = self.create_publisher(
            ObstacleDistance,
            '/fmu/in/obstacle_distance',
            qos_profile
        )
        
        self.collision_status_pub = self.create_publisher(
            Float32MultiArray,
            '/collision_prevention/status',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/collision_prevention/markers',
            10
        )
        
        # Timer for main control loop
        self.timer = self.create_timer(1.0/self.update_rate, self.control_loop)
        
        # Timer for obstacle distance publishing
        self.obstacle_timer = self.create_timer(0.1, self.publish_obstacle_distance)
        
        self.get_logger().info('Collision Prevention Node Started')
        self.get_logger().info(f'Enabled: {self.enabled}')
        self.get_logger().info(f'Min Distance: {self.min_distance}m')
        self.get_logger().info(f'Emergency Distance: {self.emergency_distance}m')
        
    def attitude_callback(self, msg):
        """Store vehicle attitude"""
        self.vehicle_attitude = msg
        
    def position_callback(self, msg):
        """Store vehicle position"""
        self.vehicle_position = msg
        
    def status_callback(self, msg):
        """Store vehicle status"""
        self.vehicle_status = msg
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        
    def laser_scan_callback(self, msg):
        """Process laser scan data"""
        if not self.enabled:
            return
            
        self.last_sensor_time = time.time()
        
        # Convert laser scan to obstacle distances
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max:
                continue
                
            angle = angle_min + i * angle_increment
            
            # Convert to body frame (assuming laser is mounted forward-facing)
            sector = int((angle + pi) * 180 / pi / 5) % 72  # Convert to 0-71 sector
            
            if distance < self.sector_distances[sector]:
                self.sector_distances[sector] = distance
                
    def pointcloud_callback(self, msg):
        """Process point cloud data for obstacle detection"""
        # Implementation would depend on your specific point cloud format
        # This is a placeholder for processing 3D obstacle data
        pass
        
    def velocity_cmd_callback(self, msg):
        """Process velocity commands and apply collision prevention"""
        if not self.enabled or not self.offboard_mode:
            # Pass through command without modification
            self.publish_trajectory(msg.twist.linear.x, msg.twist.linear.y, 
                                   msg.twist.linear.z, msg.twist.angular.z)
            return
            
        # Check sensor timeout
        if time.time() - self.last_sensor_time > self.sensor_timeout:
            self.get_logger().warn('Sensor data timeout - reducing velocity')
            # Reduce velocity when no sensor data
            safe_vx = msg.twist.linear.x * 0.3
            safe_vy = msg.twist.linear.y * 0.3
            safe_vz = msg.twist.linear.z
            self.publish_trajectory(safe_vx, safe_vy, safe_vz, msg.twist.angular.z)
            return
            
        # Apply collision prevention
        safe_velocity = self.compute_safe_velocity(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.z
        )
        
        # Publish modified trajectory
        self.publish_trajectory(
            safe_velocity[0],
            safe_velocity[1],
            safe_velocity[2],
            safe_velocity[3]
        )
        
    def compute_safe_velocity(self, vx, vy, vz, yaw_rate):
        """Compute safe velocity considering obstacles"""
        
        # Get minimum obstacle distance in direction of travel
        travel_direction = atan2(vy, vx) if (vx != 0 or vy != 0) else 0
        travel_speed = sqrt(vx**2 + vy**2)
        
        # Check obstacles in a cone around travel direction
        cone_angle = pi/6  # 30 degree cone
        min_obstacle_distance = self.max_distance
        
        for sector in range(72):
            sector_angle = sector * 5 * pi / 180
            angle_diff = abs(atan2(sin(sector_angle - travel_direction), 
                                  cos(sector_angle - travel_direction)))
            
            if angle_diff < cone_angle:
                if self.sector_distances[sector] < min_obstacle_distance:
                    min_obstacle_distance = self.sector_distances[sector]
        
        # Calculate velocity scaling factor
        scale_factor = 1.0
        
        if min_obstacle_distance < self.emergency_distance:
            # Emergency stop
            scale_factor = 0.0
            self.get_logger().warn(f'EMERGENCY STOP - Obstacle at {min_obstacle_distance:.2f}m')
            
        elif min_obstacle_distance < self.min_distance:
            # Strong reduction
            scale_factor = 0.1
            self.get_logger().warn(f'Close obstacle at {min_obstacle_distance:.2f}m - reducing velocity')
            
        elif min_obstacle_distance < self.slowdown_distance:
            # Gradual slowdown
            scale_factor = (min_obstacle_distance - self.min_distance) / (self.slowdown_distance - self.min_distance)
            scale_factor = max(self.velocity_reduction, scale_factor)
            
        # Apply scaling to horizontal velocity
        safe_vx = vx * scale_factor
        safe_vy = vy * scale_factor
        
        # Vertical velocity is not affected by horizontal obstacles
        safe_vz = vz
        
        # Yaw rate can be maintained
        safe_yaw = yaw_rate
        
        # Publish status
        self.publish_collision_status(min_obstacle_distance, scale_factor)
        
        # Publish visualization markers
        self.publish_visualization_markers()
        
        return [safe_vx, safe_vy, safe_vz, safe_yaw]
        
    def publish_trajectory(self, vx, vy, vz, yaw_rate):
        """Publish trajectory setpoint to PX4"""
        
        # Publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)
        
        # Convert body frame to world frame if attitude is available
        if self.vehicle_attitude is not None:
            q = self.vehicle_attitude.q
            yaw = atan2(2.0*(q[3]*q[0] + q[1]*q[2]), 
                       1.0 - 2.0*(q[0]*q[0] + q[1]*q[1]))
            
            cos_yaw = cos(-yaw)
            sin_yaw = sin(-yaw)
            world_vx = vx * cos_yaw - vy * sin_yaw
            world_vy = vx * sin_yaw + vy * cos_yaw
        else:
            world_vx = vx
            world_vy = vy
            
        # Create and publish trajectory message
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = world_vx
        trajectory_msg.velocity[1] = world_vy
        trajectory_msg.velocity[2] = vz
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = yaw_rate
        
        self.trajectory_pub.publish(trajectory_msg)
        
    def publish_obstacle_distance(self):
        """Publish obstacle distance message to PX4"""
        if not self.enabled:
            return
            
        # Update obstacle message
        self.obstacle_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Convert distances to centimeters and clamp to uint16 range
        for i in range(72):
            distance_cm = int(self.sector_distances[i] * 100)
            distance_cm = max(0, min(65535, distance_cm))
            self.obstacle_msg.distances[i] = distance_cm
            
        self.obstacle_distance_pub.publish(self.obstacle_msg)
        
    def publish_collision_status(self, min_distance, scale_factor):
        """Publish collision prevention status"""
        status_msg = Float32MultiArray()
        status_msg.data = [
            float(self.enabled),
            min_distance,
            scale_factor,
            float(self.offboard_mode)
        ]
        self.collision_status_pub.publish(status_msg)
        
    def publish_visualization_markers(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "base_link"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "collision_prevention"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add obstacle markers
        for i, distance in enumerate(self.sector_distances):
            if distance < self.max_distance:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = i + 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Position based on sector angle and distance
                angle = i * 5 * pi / 180
                marker.pose.position.x = distance * cos(angle)
                marker.pose.position.y = distance * sin(angle)
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Size
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                
                # Color based on distance
                if distance < self.emergency_distance:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif distance < self.min_distance:
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                elif distance < self.slowdown_distance:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    
                marker.color.a = 0.8
                marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
                
                marker_array.markers.append(marker)
                
        self.marker_pub.publish(marker_array)
        
    def control_loop(self):
        """Main control loop"""
        # Decay obstacle distances over time (simple temporal filtering)
        decay_rate = 0.95
        self.sector_distances = self.sector_distances * decay_rate + self.max_distance * (1 - decay_rate)
        
def main(args=None):
    rclpy.init(args=args)
    collision_prevention = CollisionPrevention()
    
    try:
        rclpy.spin(collision_prevention)
    except KeyboardInterrupt:
        collision_prevention.get_logger().info('Shutting down collision prevention node')
    finally:
        collision_prevention.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()