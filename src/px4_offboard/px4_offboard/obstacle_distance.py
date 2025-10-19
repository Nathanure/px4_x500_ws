#!/usr/bin/env python3
"""
Bridge Gazebo 2D Lidar to PX4 ObstacleDistance message
Enhanced with detailed logging for debugging
FIXED: Properly handles max-range readings as valid "clear space"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import ObstacleDistance
import numpy as np
import math


class LidarToObstacleDistance(Node):
    def __init__(self):
        super().__init__('obstacle_distance')
        
        # Parameters
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('sensor_frame', 'MAV_FRAME_BODY_FRD')
        self.declare_parameter('bin_count', 72)  # 360/5 = 72 bins at 5° resolution
        self.declare_parameter('max_distance', 100.0)  # meters
        self.declare_parameter('min_distance', 0.1)  # meters
        
        self.bin_count = self.get_parameter('bin_count').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        
        # QoS profiles
        qos_profile_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_profile_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        lidar_topic = self.get_parameter('lidar_topic').value
        
        # Subscriber to Gazebo lidar
        self.lidar_sub = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.lidar_callback,
            qos_profile_sensor
        )
        
        # Publisher to PX4
        self.obstacle_pub = self.create_publisher(
            ObstacleDistance,
            '/fmu/in/obstacle_distance',
            qos_profile_px4
        )
        
        # For debugging
        self.obstacle_debug_pub = self.create_publisher(
            ObstacleDistance,
            '/obstacle_distance_debug',
            10
        )
        
        self._msg_count = 0
        self._first_msg_received = False
        
        self.get_logger().info('========================================')
        self.get_logger().info('Lidar to ObstacleDistance bridge initialized')
        self.get_logger().info(f'  Subscribing to: {lidar_topic}')
        self.get_logger().info(f'  Publishing to:  /fmu/in/obstacle_distance')
        self.get_logger().info(f'  Bin count:      {self.bin_count}')
        self.get_logger().info(f'  Range limits:   {self.min_distance}m - {self.max_distance}m')
        self.get_logger().info('========================================')
        self.get_logger().info('Waiting for lidar data...')
        
        # Create a timer to check if we're receiving data
        self.check_timer = self.create_timer(5.0, self.check_data_received)
    
    def check_data_received(self):
        """Periodically check if we're receiving data"""
        if not self._first_msg_received:
            self.get_logger().warn(
                f'No lidar data received yet! '
                f'Check if topic {self.get_parameter("lidar_topic").value} exists.',
                throttle_duration_sec=5.0
            )
    
    def lidar_callback(self, msg: LaserScan):
        """Convert LaserScan to ObstacleDistance"""
        
        # Log first message details
        if not self._first_msg_received:
            self._first_msg_received = True
            self.get_logger().info('========================================')
            self.get_logger().info('✓ First lidar message received!')
            self.get_logger().info(f'  Frame:       {msg.header.frame_id}')
            self.get_logger().info(f'  Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°')
            self.get_logger().info(f'  Points:      {len(msg.ranges)}')
            self.get_logger().info(f'  Range:       {msg.range_min}m to {msg.range_max}m')
            
            # Count valid readings
            valid_count = sum(1 for r in msg.ranges if not (math.isnan(r) or math.isinf(r)))
            inf_count = sum(1 for r in msg.ranges if math.isinf(r))
            
            self.get_logger().info(f'  Valid readings: {valid_count}/{len(msg.ranges)}')
            self.get_logger().info(f'  Inf readings:   {inf_count}/{len(msg.ranges)}')
            
            if valid_count > 0:
                valid_ranges = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r))]
                self.get_logger().info(f'  Min distance:   {min(valid_ranges):.2f}m')
                self.get_logger().info(f'  Max distance:   {max(valid_ranges):.2f}m')
            
            self.get_logger().info('========================================')
        
        # Create obstacle distance message
        obstacle_msg = ObstacleDistance()
        obstacle_msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
        
        # Set frame (body frame, forward = x+)
        obstacle_msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        
        # Set sensor parameters
        obstacle_msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        obstacle_msg.min_distance = int(max(msg.range_min, self.min_distance) * 100)  # convert to cm
        obstacle_msg.max_distance = int(min(msg.range_max, self.max_distance) * 100)  # convert to cm
        
        # Calculate bin size
        obstacle_msg.increment = 360.0 / self.bin_count  # degrees per bin
        obstacle_msg.angle_offset = 0.0  # forward facing = 0°
        
        # Initialize all bins to max range
        distances = [obstacle_msg.max_distance] * self.bin_count
        
        # Convert lidar data to bins
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        valid_bins = 0
        for i, range_val in enumerate(msg.ranges):
            # CRITICAL: Don't skip inf values - they represent "no obstacle detected"
            if math.isnan(range_val):
                continue
            
            # Calculate angle for this reading (in vehicle body frame)
            # Lidar: 0° = forward, positive = CCW
            lidar_angle_rad = angle_min + i * angle_increment
            
            # Convert to degrees and wrap to [0, 360)
            # In body frame: 0° = forward (X+), 90° = left (Y+), 180° = back, 270° = right
            angle_deg = math.degrees(lidar_angle_rad)
            angle_deg = (angle_deg + 360) % 360
            
            # Find corresponding bin
            bin_index = int(angle_deg / obstacle_msg.increment) % self.bin_count
            
            # CRITICAL FIX: Handle inf and max_range properly
            if math.isinf(range_val) or range_val >= msg.range_max * 0.95:
                # No obstacle detected - set to max range
                range_cm = obstacle_msg.max_distance
                # Still count as valid data (just means clear space)
                if distances[bin_index] == obstacle_msg.max_distance:
                    valid_bins += 1
            else:
                # Convert range to cm
                range_cm = int(range_val * 100)
                
                # Clamp to sensor limits
                if range_cm < obstacle_msg.min_distance:
                    range_cm = obstacle_msg.min_distance
                
                # Count as valid obstacle detection
                if range_cm < obstacle_msg.max_distance:
                    valid_bins += 1
            
            # Keep minimum distance in each bin
            if range_cm < distances[bin_index]:
                distances[bin_index] = range_cm
        
        # Fill the distances array
        obstacle_msg.distances = distances
        
        # Publish
        self.obstacle_pub.publish(obstacle_msg)
        self.obstacle_debug_pub.publish(obstacle_msg)
        
        # Log periodically
        self._msg_count += 1
        if self._msg_count % 50 == 0:  # Every 50 messages
            min_dist = min([d for d in distances if d < obstacle_msg.max_distance] or [obstacle_msg.max_distance])
            self.get_logger().info(
                f'Publishing obstacle_distance: min={min_dist/100:.2f}m, '
                f'valid_bins={valid_bins}/{self.bin_count}, '
                f'msg_count={self._msg_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarToObstacleDistance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()