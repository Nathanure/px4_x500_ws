#!/usr/bin/env python3
"""
TF Publisher for PX4 Navigation
Publishes required TF transforms from PX4 data for Nav2
Uses VehicleOdometry instead of separate position/attitude topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from math import cos, sin

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PX4TFPublisher(Node):
    def __init__(self):
        super().__init__('px4_tf_publisher')
        
        # QoS profile for PX4 topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State variables
        self.odometry = None
        self.has_received_data = False
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to VehicleOdometry (contains both position and attitude)
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )
        
        # Publisher for odometry (Nav2 needs this)
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        # Timer to publish TF at regular intervals
        self.timer = self.create_timer(0.02, self.publish_tf)  # 50Hz
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)
        self.msg_count = 0
        
        self.get_logger().info('PX4 TF Publisher started')
        self.get_logger().info('Waiting for /fmu/out/vehicle_odometry data...')
    
    def odometry_callback(self, msg):
        """Store odometry data from PX4"""
        self.odometry = msg
        self.msg_count += 1
        
        if not self.has_received_data:
            self.has_received_data = True
            self.get_logger().info('Receiving PX4 odometry data!')
    
    def print_status(self):
        """Print status information"""
        if not self.has_received_data:
            self.get_logger().warn('Still waiting for PX4 odometry data...')
            self.get_logger().warn('Check if PX4 SITL is running and EKF2 is initialized')
        else:
            self.get_logger().info(f'Publishing TF and odometry at ~50Hz (received {self.msg_count} msgs in 2s)')
        self.msg_count = 0
    
    def publish_tf(self):
        """Publish TF transforms and odometry"""
        if self.odometry is None:
            return
        
        now = self.get_clock().now().to_msg()
        
        # Publish odom -> base_link transform
        # Convert PX4 NED to ROS ENU
        # PX4: X=North, Y=East, Z=Down
        # ROS: X=East, Y=North, Z=Up
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = now
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        
        # Position: NED -> ENU
        # PX4 VehicleOdometry uses local frame (NED)
        odom_to_base.transform.translation.x = self.odometry.position[1]   # East (Y in NED)
        odom_to_base.transform.translation.y = self.odometry.position[0]   # North (X in NED)
        odom_to_base.transform.translation.z = -self.odometry.position[2]  # Up (-Z in NED)
        
        # Attitude: quaternion NED -> ENU
        # PX4 quaternion is [w, x, y, z]
        q = self.odometry.q
        odom_to_base.transform.rotation.w = q[0]
        odom_to_base.transform.rotation.x = q[1]
        odom_to_base.transform.rotation.y = -q[2]
        odom_to_base.transform.rotation.z = -q[3]
        
        self.tf_broadcaster.sendTransform(odom_to_base)
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position (NED -> ENU)
        odom_msg.pose.pose.position.x = self.odometry.position[1]
        odom_msg.pose.pose.position.y = self.odometry.position[0]
        odom_msg.pose.pose.position.z = -self.odometry.position[2]
        
        # Orientation
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = -q[2]
        odom_msg.pose.pose.orientation.z = -q[3]
        
        # Velocity (NED -> ENU)
        # VehicleOdometry velocity is in body frame, need to rotate to world frame
        yaw = np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 
                        1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]))
        
        vx_body = self.odometry.velocity[0]  # Forward
        vy_body = self.odometry.velocity[1]  # Right
        vz_body = self.odometry.velocity[2]  # Down
        
        # Rotate to world frame
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        
        vx_world = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world = vx_body * sin_yaw + vy_body * cos_yaw
        
        # Convert NED to ENU
        odom_msg.twist.twist.linear.x = vy_world   # East velocity
        odom_msg.twist.twist.linear.y = vx_world   # North velocity
        odom_msg.twist.twist.linear.z = -vz_body   # Up velocity
        
        # Angular velocity (body frame)
        odom_msg.twist.twist.angular.x = self.odometry.angular_velocity[0]
        odom_msg.twist.twist.angular.y = -self.odometry.angular_velocity[1]
        odom_msg.twist.twist.angular.z = -self.odometry.angular_velocity[2]
        
        # Covariances (if available in VehicleOdometry)
        # For now, use default values
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()