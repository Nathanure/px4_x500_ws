#!/usr/bin/env python3
"""
PX4 Odometry Publisher for SLAM Toolbox
Publishes /odom topic (nav_msgs/Odometry) required by SLAM Toolbox
Note: TF transforms are still published by px4_offboard/tf_publisher
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import math


class PX4OdomPublisher(Node):
    def __init__(self):
        super().__init__('px4_odom_publisher')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = self.get_parameter('base_frame').value
        
        # QoS profile for PX4 (Best Effort)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to PX4 vehicle odometry
        self.vehicle_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odom_callback,
            qos_profile
        )
        
        # Publish ROS standard odometry (Reliable QoS for SLAM)
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.get_logger().info('PX4 Odometry Publisher started')
        self.get_logger().info(f'Publishing /odom topic for SLAM Toolbox')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info('Note: TF transforms handled by px4_offboard/tf_publisher')
        
        self.msg_count = 0
    
    def vehicle_odom_callback(self, msg):
        """
        Convert PX4 VehicleOdometry (NED frame) to ROS Odometry (ENU frame)
        Only publishes the message - TF is handled elsewhere
        """
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.base_frame
        
        # Convert NED to ENU
        # PX4: x=North, y=East, z=Down
        # ROS: x=East, y=North, z=Up
        odom.pose.pose.position.x = float(msg.position[1])   # East
        odom.pose.pose.position.y = float(msg.position[0])   # North
        odom.pose.pose.position.z = float(-msg.position[2])  # Up

        # Quaternion conversion (NED to ENU)
        # This is a simplified conversion - you may need to adjust based on your setup
        odom.pose.pose.orientation.w = float(msg.q[0])
        odom.pose.pose.orientation.x = float(msg.q[1])
        odom.pose.pose.orientation.y = float(msg.q[2])
        odom.pose.pose.orientation.z = float(msg.q[3])
        
        # Convert velocity NED to ENU
        odom.twist.twist.linear.x = float(msg.velocity[1])   # East
        odom.twist.twist.linear.y = float(msg.velocity[0])   # North
        odom.twist.twist.linear.z = float(-msg.velocity[2])  # Up
        
        # Angular velocity (body frame - same for NED/ENU)
        odom.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom.twist.twist.angular.z = float(msg.angular_velocity[2])
        
        # Set covariances from PX4
        # Position covariance (diagonal elements)
        odom.pose.covariance[0] = float(msg.position_variance[1])   # x
        odom.pose.covariance[7] = float(msg.position_variance[0])   # y
        odom.pose.covariance[14] = float(msg.position_variance[2])  # z
        
        # Orientation covariance
        odom.pose.covariance[21] = float(msg.orientation_variance[0])  # roll
        odom.pose.covariance[28] = float(msg.orientation_variance[1])  # pitch
        odom.pose.covariance[35] = float(msg.orientation_variance[2])  # yaw
        
        # Velocity covariance
        odom.twist.covariance[0] = float(msg.velocity_variance[1])   # vx
        odom.twist.covariance[7] = float(msg.velocity_variance[0])   # vy
        odom.twist.covariance[14] = float(msg.velocity_variance[2])  # vz
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Periodic logging
        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'/odom published: pos=[{odom.pose.pose.position.x:.2f}, '
                f'{odom.pose.pose.position.y:.2f}], '
                f'vel=[{odom.twist.twist.linear.x:.2f}, {odom.twist.twist.linear.y:.2f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()