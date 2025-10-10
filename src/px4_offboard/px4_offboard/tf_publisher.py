#!/usr/bin/env python3
"""
TF Publisher for PX4 X500 with 2D LiDAR
Publishes the complete transform tree: odom -> base_footprint -> base_link -> sensor_links
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from scipy.spatial.transform import Rotation


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
        
        # Create TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to PX4 position (try both topic versions)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        
        self.position_sub_v1 = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        
        # State variables
        self.current_position = None
        self.current_attitude = None
        self.position_received = False
        self.attitude_received = False
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Timer for publishing dynamic transforms
        self.timer = self.create_timer(0.02, self.publish_dynamic_transforms)  # 50Hz
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('PX4 TF Publisher Started')
        self.get_logger().info('Publishing TF tree: odom -> base_footprint -> base_link -> sensor_links')
        
    def print_status(self):
        """Print status of received messages"""
        if not self.position_received:
            self.get_logger().warn('No vehicle position data received yet!')
            self.get_logger().warn('Checking topics: /fmu/out/vehicle_local_position and /fmu/out/vehicle_local_position_v1')
        if not self.attitude_received:
            self.get_logger().warn('No vehicle attitude data received yet!')
        
    def publish_static_transforms(self):
        """Publish static transforms for the robot structure"""
        
        static_transforms = []
        
        # base_footprint to base_link (base_link is ABOVE base_footprint)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # base_link is 10cm above base_footprint
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to lidar_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.084  # Height of lidar above base_link
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to camera_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to imu_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # Publish all static transforms
        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.get_logger().info(f'Published {len(static_transforms)} static transforms')
        
    def position_callback(self, msg):
        """Store current position from PX4"""
        if not self.position_received:
            self.get_logger().info('First position message received!')
            self.position_received = True
        self.current_position = msg
        
    def attitude_callback(self, msg):
        """Store current attitude from PX4"""
        if not self.attitude_received:
            self.get_logger().info('First attitude message received!')
            self.attitude_received = True
        self.current_attitude = msg
        
    def publish_dynamic_transforms(self):
        """Publish dynamic transform from odom to base_footprint"""
        
        if self.current_position is None or self.current_attitude is None:
            return
        
        # Create transform from odom to base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Position (NED to ENU conversion)
        # PX4 uses NED (North-East-Down), ROS uses ENU (East-North-Up)
        # NED to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        t.transform.translation.x = self.current_position.y
        t.transform.translation.y = self.current_position.x
        t.transform.translation.z = -self.current_position.z
        
        # Orientation (quaternion from PX4)
        # PX4 quaternion is [w, x, y, z], need to convert NED to ENU
        q = self.current_attitude.q
        
        # Convert NED quaternion to ENU quaternion
        q_ned = np.array([q[0], q[1], q[2], q[3]])  # [w, x, y, z]
        
        # Create rotation from NED to ENU
        r_ned = Rotation.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x,y,z,w]
        r_ned_to_enu = Rotation.from_euler('z', 90, degrees=True)
        r_enu = r_ned_to_enu * r_ned
        
        q_enu = r_enu.as_quat()  # Returns [x, y, z, w]
        
        t.transform.rotation.x = q_enu[0]
        t.transform.rotation.y = q_enu[1]
        t.transform.rotation.z = q_enu[2]
        t.transform.rotation.w = q_enu[3]
        
        # Publish transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    tf_publisher = PX4TFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        tf_publisher.get_logger().info('Shutting down TF publisher')
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()