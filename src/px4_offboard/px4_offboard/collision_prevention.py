#!/usr/bin/env python3
"""
Collision Prevention for PX4 Offboard Mode
FIXED: Properly handles max-range as valid clear space, only limits when moving TOWARD obstacles
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from px4_msgs.msg import ObstacleDistance, VehicleAttitude
from std_msgs.msg import Bool
import math

class CollisionPrevention(Node):
    def __init__(self):
        super().__init__('collision_prevention')
        
        # Parameters (matching PX4 parameters)
        self.declare_parameter('cp_dist', 3.0)  # Min distance to keep from obstacles (meters)
        self.declare_parameter('cp_delay', 0.4)  # Sensor + controller delay (seconds)
        self.declare_parameter('cp_guide_ang', 30.0)  # Guidance angle (degrees)
        self.declare_parameter('cp_go_no_data', False)  # Allow movement where no data
        self.declare_parameter('mpc_xy_p', 0.95)  # Position controller P gain
        self.declare_parameter('mpc_jerk_max', 8.0)  # Max jerk (m/s³)
        self.declare_parameter('mpc_acc_hor', 3.0)  # Max horizontal acceleration (m/s²)
        self.declare_parameter('mpc_xy_vel_p_acc', 1.8)  # Velocity P gain for acceleration
        self.declare_parameter('mpc_vel_manual', 10.0)  # Max manual velocity (m/s)
        
        # Obstacle map configuration
        self.BIN_SIZE = 5  # degrees
        self.BIN_COUNT = 72  # 360 / 5
        
        # Initialize obstacle map
        self.obstacle_map = {
            'distances': [np.inf] * self.BIN_COUNT,
            'timestamps': [0] * self.BIN_COUNT,
            'max_ranges': [0] * self.BIN_COUNT,
            'min_distance': 100,  # cm
            'max_distance': 10000,  # cm
            'angle_offset': 0.0
        }
        
        # State variables
        self.vehicle_yaw = 0.0
        self.vehicle_attitude = [1.0, 0.0, 0.0, 0.0]  # quaternion
        self.is_active = True  # Always active
        self.closest_dist = np.inf
        self.closest_dist_dir = np.array([0.0, 0.0])
        self.data_received = False
        
        # Tracking for debugging
        self.last_input_vel = np.array([0.0, 0.0])
        self.last_output_vel = np.array([0.0, 0.0])
        
        # Timeout constants
        self.RANGE_STREAM_TIMEOUT = 2.0  # seconds
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        
        # Subscribe to raw obstacle distance (since fused is not available)
        self.obstacle_raw_sub = self.create_subscription(
            ObstacleDistance,
            '/obstacle_distance_debug',
            self.obstacle_callback,
            qos_profile
        )

        self.velocity_cmd_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.velocity_cmd_callback,
            qos_profile
        )
        
        # Publishers
        self.constrained_velocity_pub = self.create_publisher(
            Twist,
            '/constrained_velocity_cmd',
            qos_profile
        )
        
        self.collision_prevention_active_pub = self.create_publisher(
            Bool,
            '/collision_prevention_active',
            qos_profile
        )
        
        self.get_logger().info('========================================')
        self.get_logger().info('Collision Prevention Node Initialized')
        self.get_logger().info(f'  cp_dist: {self.get_parameter("cp_dist").value}m')
        self.get_logger().info(f'  cp_go_no_data: {self.get_parameter("cp_go_no_data").value}')
        self.get_logger().info(f'  Subscribing to: /offboard_velocity_cmd')
        self.get_logger().info(f'  Publishing to: /constrained_velocity_cmd')
        self.get_logger().info('========================================')
    
        # Timer for status publishing
        self.status_timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        """Publish status for debugging"""
        valid_data = self._has_valid_data()
        valid_bins = self._count_valid_bins()
        
        # Count actual obstacles vs clear space
        obstacle_count = 0
        clear_count = 0
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for i in range(self.BIN_COUNT):
            if (current_time - self.obstacle_map['timestamps'][i]) < self.RANGE_STREAM_TIMEOUT:
                dist = self.obstacle_map['distances'][i]
                if not np.isinf(dist) and dist > 0:
                    obstacle_count += 1
                else:
                    clear_count += 1
        
        # Find obstacles in front (±45°)
        front_obstacles = []
        for i in range(self.BIN_COUNT):
            angle = (i * self.BIN_SIZE + self.obstacle_map['angle_offset']) % 360
            if angle <= 45 or angle >= 315:  # Front sector
                dist = self.obstacle_map['distances'][i] / 100.0
                if not np.isinf(dist) and dist < 20:
                    front_obstacles.append(dist)
        
        min_front = min(front_obstacles) if front_obstacles else float('inf')
        
        self.get_logger().info(
            f'CP STATUS: closest={self.closest_dist:.2f}m, front_min={min_front:.2f}m, '
            f'obstacles={obstacle_count}, clear={clear_count}, valid={valid_bins}/{self.BIN_COUNT}, '
            f'data_ok={valid_data}, '
            f'last_in=[{self.last_input_vel[0]:.2f},{self.last_input_vel[1]:.2f}], '
            f'last_out=[{self.last_output_vel[0]:.2f},{self.last_output_vel[1]:.2f}]'
        )

    def attitude_callback(self, msg):
        """Update vehicle attitude and yaw"""
        try:
            self.vehicle_attitude = msg.q
            # Calculate yaw from quaternion (NED frame)
            self.vehicle_yaw = np.arctan2(
                2.0 * (msg.q[3] * msg.q[0] + msg.q[1] * msg.q[2]),
                1.0 - 2.0 * (msg.q[0]**2 + msg.q[1]**2)
            )
        except Exception as e:
            self.get_logger().error(f'Error in attitude_callback: {e}')
    
    def obstacle_callback(self, msg):
        """Update obstacle map from sensor data"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.data_received = True
            
            # Update obstacle map
            obstacle_count = 0
            clear_count = 0
            
            for i in range(min(len(msg.distances), self.BIN_COUNT)):
                dist_cm = msg.distances[i]
                
                # CRITICAL FIX: Distinguish between "max range" (valid but no obstacle) 
                # and truly invalid data
                if dist_cm >= msg.max_distance * 0.95:  # Within 5% of max range = no obstacle
                    # Valid data, but no obstacle detected in this direction
                    self.obstacle_map['distances'][i] = np.inf  # No obstacle
                    self.obstacle_map['timestamps'][i] = current_time  # Mark as recent
                    self.obstacle_map['max_ranges'][i] = msg.max_distance
                    clear_count += 1
                elif dist_cm > 0 and dist_cm < msg.max_distance:
                    # Valid obstacle detected
                    self.obstacle_map['distances'][i] = dist_cm
                    self.obstacle_map['timestamps'][i] = current_time
                    self.obstacle_map['max_ranges'][i] = msg.max_distance
                    obstacle_count += 1
                else:
                    # Truly invalid data (negative, zero)
                    self.obstacle_map['distances'][i] = np.inf
                    # Don't update timestamp - let it become stale
            
            self.obstacle_map['min_distance'] = msg.min_distance
            self.obstacle_map['max_distance'] = msg.max_distance
            self.obstacle_map['angle_offset'] = msg.angle_offset
            
            # Update closest obstacle
            self._update_closest_obstacle()
            
            # Log first message
            if not hasattr(self, '_first_obstacle_logged'):
                self._first_obstacle_logged = True
                self.get_logger().info(
                    f'✓ Receiving obstacle data: {obstacle_count} obstacles detected, '
                    f'{clear_count} clear directions'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in obstacle_callback: {e}')
    
    def _update_closest_obstacle(self):
        """Find closest obstacle and its direction"""
        try:
            self.closest_dist = np.inf
            self.closest_dist_dir = np.array([0.0, 0.0])
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            for i in range(self.BIN_COUNT):
                # Check if data is recent
                if (current_time - self.obstacle_map['timestamps'][i]) > self.RANGE_STREAM_TIMEOUT:
                    continue
                
                distance_m = self.obstacle_map['distances'][i] / 100.0  # Convert cm to m
                
                # Skip invalid distances
                if np.isinf(distance_m) or distance_m <= 0 or distance_m > 100:
                    continue
                
                if distance_m < self.closest_dist:
                    self.closest_dist = distance_m
                    
                    # Calculate direction in body frame
                    angle_deg = i * self.BIN_SIZE + self.obstacle_map['angle_offset']
                    angle_rad = math.radians(angle_deg)
                    
                    # Direction in body frame (0° = forward, 90° = left, 180° = back, 270° = right)
                    self.closest_dist_dir = np.array([np.cos(angle_rad), np.sin(angle_rad)])
                    
        except Exception as e:
            self.get_logger().error(f'Error in _update_closest_obstacle: {e}')
    
    def velocity_cmd_callback(self, msg):
        """Process velocity command and apply collision prevention"""
        try:
            # Extract velocity command
            original_vel = np.array([msg.linear.x, msg.linear.y])
            self.last_input_vel = original_vel
            
            # Apply collision prevention
            modified_vel = self._modify_velocity_setpoint(original_vel)
            self.last_output_vel = modified_vel
            
            # Create constrained message
            constrained_msg = Twist()
            constrained_msg.linear.x = float(modified_vel[0])  # CRITICAL: Convert to Python float
            constrained_msg.linear.y = float(modified_vel[1])  # CRITICAL: Convert to Python float
            constrained_msg.linear.z = msg.linear.z  # Keep vertical unchanged
            constrained_msg.angular.z = msg.angular.z
            
            # Publish constrained velocity FIRST (most important)
            self.constrained_velocity_pub.publish(constrained_msg)
            
            # Publish active status (with proper bool conversion)
            try:
                active_msg = Bool()
                vel_diff = np.linalg.norm(modified_vel - original_vel)
                active_msg.data = bool(vel_diff > 0.01)  # CRITICAL: Convert numpy bool to Python bool
                self.collision_prevention_active_pub.publish(active_msg)
            except Exception as bool_error:
                self.get_logger().warn(f'Error publishing active status: {bool_error}')
                # Don't let this stop the main functionality
            
            # Log when there's velocity input
            if np.linalg.norm(original_vel) > 0.01:
                # Calculate angle of velocity and closest obstacle for debugging
                vel_angle = np.arctan2(original_vel[1], original_vel[0]) * 180 / np.pi
                closest_angle = np.arctan2(self.closest_dist_dir[1], self.closest_dist_dir[0]) * 180 / np.pi
                
                self.get_logger().info(
                    f'VEL_CB: in=[{original_vel[0]:.2f}, {original_vel[1]:.2f}] (angle={vel_angle:.0f}°), '
                    f'out=[{modified_vel[0]:.2f}, {modified_vel[1]:.2f}], '
                    f'closest={self.closest_dist:.2f}m (angle={closest_angle:.0f}°)',
                    throttle_duration_sec=0.5
                )
            
        except Exception as e:
            self.get_logger().error(f'CRITICAL Error in velocity_cmd_callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            # CRITICAL FIX: On error, STOP instead of passing through original command
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = msg.linear.z
            stop_msg.angular.z = msg.angular.z
            self.constrained_velocity_pub.publish(stop_msg)
            self.get_logger().warn('Emergency stop published due to error')
    
    def _modify_velocity_setpoint(self, velocity_cmd):
        """
        Modify velocity setpoint based on obstacle distances
        SIMPLIFIED AND MORE AGGRESSIVE VERSION
        """
        try:
            vel_norm = np.linalg.norm(velocity_cmd)
            
            # CRITICAL FIX: Clamp insane velocities
            max_vel = 10.0  # 10 m/s max
            if vel_norm > max_vel:
                self.get_logger().warn(
                    f'INSANE VELOCITY DETECTED: {vel_norm:.2f} m/s! Clamping to {max_vel} m/s. '
                    f'Check your control input!',
                    throttle_duration_sec=1.0
                )
                velocity_cmd = velocity_cmd / vel_norm * max_vel
                vel_norm = max_vel
            
            # No velocity command - pass through
            if vel_norm < 0.001:
                return velocity_cmd
            
            # Check if we have valid obstacle data
            if not self._has_valid_data():
                cp_go_no_data = self.get_parameter('cp_go_no_data').value
                if not cp_go_no_data:
                    self.get_logger().warn('No valid obstacle data - stopping', throttle_duration_sec=2.0)
                    return np.array([0.0, 0.0])
                return velocity_cmd
            
            cp_dist = self.get_parameter('cp_dist').value
            
            # CRITICAL: If too close, check if we're trying to move AWAY
            if self.closest_dist < cp_dist * 0.5:  # Within 1.5m (half of 3m)
                # Check if moving away from closest obstacle
                vel_toward_closest = np.dot(velocity_cmd, self.closest_dist_dir)
                
                # Detailed debugging
                vel_angle = np.arctan2(velocity_cmd[1], velocity_cmd[0]) * 180 / np.pi
                obstacle_angle = np.arctan2(self.closest_dist_dir[1], self.closest_dist_dir[0]) * 180 / np.pi
                angle_diff = abs(vel_angle - obstacle_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                
                self.get_logger().warn(
                    f'EMERGENCY ZONE: dist={self.closest_dist:.2f}m, '
                    f'vel=[{velocity_cmd[0]:.1f},{velocity_cmd[1]:.1f}] angle={vel_angle:.0f}°, '
                    f'obstacle_dir=[{self.closest_dist_dir[0]:.2f},{self.closest_dist_dir[1]:.2f}] angle={obstacle_angle:.0f}°, '
                    f'vel_toward={vel_toward_closest:.2f}, angle_diff={angle_diff:.0f}°',
                    throttle_duration_sec=0.3
                )
                
                if vel_toward_closest < -0.05:  # Negative = moving AWAY from closest obstacle
                    self.get_logger().info(
                        f'  → ALLOWING ESCAPE (vel_toward={vel_toward_closest:.2f} < -0.05)',
                        throttle_duration_sec=0.3
                    )
                    return velocity_cmd  # Allow escape!
                else:
                    self.get_logger().warn(
                        f'  → BLOCKING (vel_toward={vel_toward_closest:.2f} >= -0.05, not moving away enough)',
                        throttle_duration_sec=0.3
                    )
                    return np.array([0.0, 0.0])
            
            # If obstacle is close, check all directions
            if self.closest_dist < cp_dist * 2:
                # Check if moving toward ANY close obstacle (more aggressive)
                scale_factor = self._calculate_velocity_scale_all_directions(velocity_cmd, cp_dist)
                
                if scale_factor < 0.99:  # Any reduction at all
                    modified_vel = velocity_cmd * scale_factor
                    
                    # Zero out very small velocities
                    if np.linalg.norm(modified_vel) < 0.05:
                        modified_vel = np.array([0.0, 0.0])
                    
                    self.get_logger().info(
                        f'CP LIMITING: dist={self.closest_dist:.2f}m, scale={scale_factor:.3f}, '
                        f'in=[{velocity_cmd[0]:.2f},{velocity_cmd[1]:.2f}], '
                        f'out=[{modified_vel[0]:.2f},{modified_vel[1]:.2f}]',
                        throttle_duration_sec=0.3
                    )
                    return modified_vel
                else:
                    # Scale is 1.0 - not limiting (moving away or parallel)
                    self.get_logger().info(
                        f'CP NOT LIMITING: dist={self.closest_dist:.2f}m, scale=1.00 (moving away/parallel)',
                        throttle_duration_sec=0.5
                    )
            
            return velocity_cmd
            
        except Exception as e:
            self.get_logger().error(f'Error in _modify_velocity_setpoint: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            # On error, STOP
            return np.array([0.0, 0.0])
    
    def _calculate_velocity_scale_all_directions(self, velocity_cmd, cp_dist):
        """
        Check obstacles in ALL directions and calculate most restrictive scale
        ONLY limits when moving TOWARD obstacles, allows free movement AWAY from them
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        vel_norm = np.linalg.norm(velocity_cmd)
        
        if vel_norm < 0.001:
            return 1.0
        
        min_scale = 1.0
        obstacles_in_direction = False
        limiting_bins = []  # Track which bins are limiting us
        
        # Check each bin
        for i in range(self.BIN_COUNT):
            # Skip stale data
            if (current_time - self.obstacle_map['timestamps'][i]) > self.RANGE_STREAM_TIMEOUT:
                continue
            
            distance_m = self.obstacle_map['distances'][i] / 100.0
            
            # Skip invalid distances
            if np.isinf(distance_m) or distance_m <= 0 or distance_m > 100:
                continue
            
            # Only check obstacles within range
            if distance_m > cp_dist * 2:
                continue
            
            # Calculate direction to this obstacle
            angle_deg = i * self.BIN_SIZE + self.obstacle_map['angle_offset']
            angle_rad = math.radians(angle_deg)
            obstacle_dir = np.array([np.cos(angle_rad), np.sin(angle_rad)])
            
            # Calculate velocity component toward this obstacle
            vel_toward = np.dot(velocity_cmd, obstacle_dir)
            
            # CRITICAL FIX: Only limit if moving TOWARD obstacle (positive dot product)
            # If negative, we're moving AWAY - allow full speed!
            if vel_toward > 0.05:  # Small threshold to avoid numerical issues
                obstacles_in_direction = True
                
                # Calculate scale based on distance
                # Zero at cp_dist, 1.0 at cp_dist*2
                distance_ratio = (distance_m - cp_dist) / cp_dist
                distance_ratio = np.clip(distance_ratio, 0.0, 1.0)
                
                # More aggressive scaling: cubic instead of quadratic
                scale = distance_ratio ** 2
                
                # Keep the most restrictive scale
                if scale < min_scale:
                    min_scale = scale
                    limiting_bins.append((i, angle_deg, distance_m, vel_toward))
        
        # Debug logging when limiting
        if obstacles_in_direction and min_scale < 0.99:
            most_limiting = limiting_bins[-1] if limiting_bins else None
            if most_limiting:
                bin_idx, angle, dist, vel_comp = most_limiting
                self.get_logger().info(
                    f'  Most limiting: bin={bin_idx}, angle={angle:.0f}°, dist={dist:.2f}m, vel_toward={vel_comp:.2f}',
                    throttle_duration_sec=0.5
                )
        
        # If no obstacles in direction of movement, return full speed
        if not obstacles_in_direction:
            return 1.0
        
        return min_scale
    
    def _has_valid_data(self):
        """Check if we have recent valid obstacle data"""
        try:
            if not self.data_received:
                return False
            
            valid_bins = self._count_valid_bins()
            
            # Require at least 5% of bins to have valid data
            return valid_bins >= (self.BIN_COUNT * 0.05)
            
        except Exception as e:
            self.get_logger().error(f'Error in _has_valid_data: {e}')
            return False
    
    def _count_valid_bins(self):
        """Count bins with valid recent data (including max range = clear)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        valid_bins = 0
        
        for i in range(self.BIN_COUNT):
            # Check if data is recent
            if (current_time - self.obstacle_map['timestamps'][i]) < self.RANGE_STREAM_TIMEOUT:
                # CRITICAL FIX: Both obstacles AND clear directions count as valid
                # We only care that we have RECENT data
                valid_bins += 1
        
        return valid_bins


def main(args=None):
    rclpy.init(args=args)
    collision_prevention = CollisionPrevention()
    
    try:
        rclpy.spin(collision_prevention)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        collision_prevention.get_logger().error(f'Collision prevention crashed: {e}')
    finally:
        collision_prevention.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()