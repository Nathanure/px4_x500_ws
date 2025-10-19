#!/usr/bin/env python3
"""
TF Publisher for PX4 X500 with 2D LiDAR
ROBUST: Publishes immediately regardless of clock state
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time


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
        
        # Get use_sim_time parameter
        try:
            use_sim = self.get_parameter('use_sim_time').value
        except:
            use_sim = False
        
        self.get_logger().info('========================================')
        self.get_logger().info('PX4 TF Publisher Started')
        self.get_logger().info(f'use_sim_time: {use_sim}')
        self.get_logger().info('Publishing TF tree: odom -> base_footprint -> base_link -> sensor_links')
        self.get_logger().info('========================================')
        
        # Publish static transforms IMMEDIATELY
        # This works because we use get_current_time() which handles both cases
        self.publish_static_transforms()
        self.get_logger().info('✓ Initial static transforms published')
        
        # Timer for publishing dynamic transforms (50Hz)
        self.dynamic_timer = self.create_timer(0.02, self.publish_dynamic_transforms)
        
        # Timer to re-publish static transforms periodically (1Hz)
        self.static_timer = self.create_timer(1.0, self.publish_static_transforms)
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.print_status)
        
    def get_current_time(self):
        """
        Get current time that works with both sim time and wall time.
        Returns the actual clock time, or None if not ready.
        """
        now = self.get_clock().now()
        
        # For sim time, we need a valid non-zero timestamp
        # But we should use the actual sim time once it starts
        if now.nanoseconds == 0:
            # Clock not initialized yet - return None to skip publishing
            return None
        
        return now.to_msg()
    
    def get_static_time(self):
        """
        Get time for static transforms - can use fallback if needed.
        """
        now = self.get_clock().now()
        
        # If time is zero (sim clock not started yet), use a valid non-zero time
        # Static transforms can tolerate this
        if now.nanoseconds == 0:
            # Return epoch + 1 second (valid but old timestamp)
            time_msg = Time()
            time_msg.sec = 1
            time_msg.nanosec = 0
            return time_msg
        
        return now.to_msg()
        
    def print_status(self):
        """Print status of received messages"""
        status_parts = []
        
        if self.position_received:
            status_parts.append('✓ Position')
        else:
            status_parts.append('✗ Position')
            
        if self.attitude_received:
            status_parts.append('✓ Attitude')
        else:
            status_parts.append('✗ Attitude')
        
        # Check clock status
        now = self.get_clock().now()
        if now.nanoseconds > 0:
            status_parts.append(f'✓ Clock ({now.nanoseconds/1e9:.1f}s)')
        else:
            status_parts.append('⚠ Clock (0s)')
        
        self.get_logger().info(f'TF Status: {" | ".join(status_parts)}')
        
        if not self.position_received:
            self.get_logger().warn('No vehicle position! Check: /fmu/out/vehicle_local_position*')
        if not self.attitude_received:
            self.get_logger().warn('No vehicle attitude! Check: /fmu/out/vehicle_attitude')
        
    def publish_static_transforms(self):
        """Publish static transforms for the robot structure"""
        
        # Get current time (handles both sim and wall clock, with fallback)
        current_time = self.get_static_time()
        
        static_transforms = []
        
        # base_footprint to base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to lidar_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.084
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to camera_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # base_link to imu_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # Publish all static transforms
        self.static_tf_broadcaster.sendTransform(static_transforms)
        
    def position_callback(self, msg):
        """Store current position from PX4"""
        if not self.position_received:
            self.get_logger().info('✓ First position message received!')
            self.position_received = True
        self.current_position = msg
        
    def attitude_callback(self, msg):
        """Store current attitude from PX4"""
        if not self.attitude_received:
            self.get_logger().info('✓ First attitude message received!')
            self.attitude_received = True
        self.current_attitude = msg
        
    def publish_dynamic_transforms(self):
        """Publish dynamic transform from odom to base_footprint"""
        
        # Need both position and attitude
        if self.current_position is None or self.current_attitude is None:
            return
        
        # Get current time - must be valid for dynamic transforms
        current_time = self.get_current_time()
        
        # Skip if clock not ready (returns None)
        if current_time is None:
            if not hasattr(self, '_clock_wait_logged'):
                self._clock_wait_logged = True
                self.get_logger().warn('Waiting for valid clock to publish dynamic transforms...')
            return
        
        # Create transform from odom to base_footprint
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Position (NED to ENU conversion)
        t.transform.translation.x = self.current_position.y
        t.transform.translation.y = self.current_position.x
        t.transform.translation.z = -self.current_position.z
        
        # Orientation (NED to ENU conversion)
        q = self.current_attitude.q
        r_ned = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        r_ned_to_enu = Rotation.from_euler('z', 90, degrees=True)
        r_enu = r_ned_to_enu * r_ned
        q_enu = r_enu.as_quat()
        
        t.transform.rotation.x = q_enu[0]
        t.transform.rotation.y = q_enu[1]
        t.transform.rotation.z = q_enu[2]
        t.transform.rotation.w = q_enu[3]
        
        # Publish transform
        self.tf_broadcaster.sendTransform(t)
        
        # Log only first time
        if not hasattr(self, '_dynamic_published'):
            self._dynamic_published = True
            clock_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f'✓ Publishing dynamic odom->base_footprint transform (clock={clock_time:.2f}s)')
        
        # Periodic logging to verify it's still publishing
        if not hasattr(self, '_dynamic_count'):
            self._dynamic_count = 0
        self._dynamic_count += 1
        
        if self._dynamic_count % 250 == 0:  # Every 5 seconds at 50Hz
            self.get_logger().info(
                f'Dynamic TF still publishing: {self._dynamic_count} frames sent',
                throttle_duration_sec=5.0
            )


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