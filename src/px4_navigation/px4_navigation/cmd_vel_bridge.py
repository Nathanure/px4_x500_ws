#!/usr/bin/env python3
"""
cmd_vel Bridge Node for PX4 X500 Drone
Converts Nav2 Twist messages to PX4 OffboardControlMode + TrajectorySetpoint

CRITICAL: This bridge must handle holonomic velocity commands!
Nav2 outputs: linear.x, linear.y, angular.z (BODY FRAME)
PX4 expects: velocity in NED frame (North-East-Down) (WORLD FRAME)

ALTITUDE HOLD: When no commands received, maintains current altitude instead of descending

FIXED: Now properly converts body-frame velocities to world-frame using yaw
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleAttitude
import numpy as np


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Parameters
        self.declare_parameter('enable_bridge', True)
        self.declare_parameter('velocity_scale_x', 1.0)
        self.declare_parameter('velocity_scale_y', 1.0)
        self.declare_parameter('yaw_rate_scale', 1.0)
        self.declare_parameter('command_timeout', 0.5)  # Stop if no cmd for 0.5s
        
        self.enable_bridge = self.get_parameter('enable_bridge').value
        self.vel_scale_x = self.get_parameter('velocity_scale_x').value
        self.vel_scale_y = self.get_parameter('velocity_scale_y').value
        self.yaw_rate_scale = self.get_parameter('yaw_rate_scale').value
        self.cmd_timeout = self.get_parameter('command_timeout').value
        
        # QoS profile for PX4 messages (must match PX4 settings)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        # NEW: Subscribe to vehicle position for altitude hold
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )
        
        # NEW: Subscribe to vehicle attitude for yaw
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        # State variables
        self.last_cmd_time = self.get_clock().now()
        self.vehicle_status = None
        self.offboard_mode = False
        
        # Altitude hold variables
        self.current_altitude_ned = None  # Current Z position in NED (Down is positive)
        self.hover_altitude_ned = None    # Target altitude for hovering
        self.altitude_initialized = False
        self.first_cmd_received = False
        
        # NEW: Yaw tracking for body-to-world conversion
        self.current_yaw = 0.0  # Current yaw angle in radians (NED frame)
        self.yaw_initialized = False
        
        # Timer to publish offboard control mode at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('cmd_vel Bridge Node Started (WITH ALTITUDE HOLD + BODY-TO-WORLD CONVERSION)')
        self.get_logger().info(f'Bridge enabled: {self.enable_bridge}')
        self.get_logger().info(f'Velocity scales - X: {self.vel_scale_x}, Y: {self.vel_scale_y}, Yaw: {self.yaw_rate_scale}')
        self.get_logger().info('FIXED: Body-frame velocities will be transformed to world frame using yaw')
    
    def vehicle_status_callback(self, msg):
        """Monitor vehicle status"""
        self.vehicle_status = msg
        # Check if in offboard mode (mode 14)
        self.offboard_mode = (msg.nav_state == 14)
    
    def vehicle_position_callback(self, msg):
        """Store current vehicle position for altitude hold"""
        # Store current altitude (NED frame: positive = down)
        self.current_altitude_ned = msg.z
        
        # Initialize hover altitude on first position update when in offboard mode
        if not self.altitude_initialized and self.offboard_mode:
            self.hover_altitude_ned = self.current_altitude_ned
            self.altitude_initialized = True
            self.get_logger().info(f'Altitude hold initialized: {-self.hover_altitude_ned:.2f}m above ground (NED Z={self.hover_altitude_ned:.2f})')
    
    def vehicle_attitude_callback(self, msg):
        """Extract yaw from vehicle attitude quaternion"""
        # PX4 quaternion format: [w, x, y, z]
        q = msg.q
        
        # Convert quaternion to yaw (NED frame)
        # Yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
        self.current_yaw = np.arctan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2)
        )
        
        if not self.yaw_initialized:
            self.yaw_initialized = True
            self.get_logger().info(f'Yaw initialized: {np.degrees(self.current_yaw):.1f}°')
    
    def cmd_vel_callback(self, msg):
        """
        Convert Nav2 Twist (body frame) to PX4 TrajectorySetpoint (NED world frame)
        
        Nav2 Convention (body frame):
        - linear.x: forward velocity (m/s)
        - linear.y: lateral velocity (m/s) - LEFT positive
        - angular.z: yaw rate (rad/s) - CCW positive
        
        PX4 Convention (NED world frame):
        - velocity[0]: North velocity (m/s)
        - velocity[1]: East velocity (m/s)
        - velocity[2]: Down velocity (always 0 for 2D navigation)
        - yaw rate: CCW positive (same as Nav2)
        
        CRITICAL: Transforms body-frame velocities to world frame using current yaw
        """
        if not self.enable_bridge:
            return
        
        # Wait for yaw to be initialized
        if not self.yaw_initialized:
            if not hasattr(self, '_yaw_wait_logged'):
                self._yaw_wait_logged = True
                self.get_logger().warn('Waiting for vehicle attitude (yaw) before processing commands...')
            return
        
        # Mark that we've received our first command
        if not self.first_cmd_received:
            self.first_cmd_received = True
            self.get_logger().info('First Nav2 command received - bridge now active')
            
            # Lock in current altitude for hovering
            if self.current_altitude_ned is not None:
                self.hover_altitude_ned = self.current_altitude_ned
                self.get_logger().info(f'Hover altitude locked: {-self.hover_altitude_ned:.2f}m above ground')
        
        # Update hover altitude continuously while receiving commands
        # This allows manual altitude changes via control.py to be tracked
        if self.current_altitude_ned is not None:
            self.hover_altitude_ned = self.current_altitude_ned
        
        self.last_cmd_time = self.get_clock().now()
        
        # Get body-frame velocities from Nav2
        vx_body = msg.linear.x * self.vel_scale_x   # Forward
        vy_body = msg.linear.y * self.vel_scale_y   # Left (positive)
        
        # CRITICAL FIX: Transform body-frame velocities to NED world frame
        # Rotation matrix from body to NED:
        # [ cos(yaw)  -sin(yaw) ] [ vx_body ]   [ vx_ned ]
        # [ sin(yaw)   cos(yaw) ] [ vy_body ] = [ vy_ned ]
        
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # NOTE: Nav2 uses left-positive for Y, but we need to account for this
        # Body frame: X=forward, Y=left
        # We want: forward in body frame -> North in NED, left in body frame -> West (negative East) in NED
        vx_ned = vx_body * cos_yaw - vy_body * sin_yaw  # North velocity
        vy_ned = vx_body * sin_yaw + vy_body * cos_yaw  # East velocity
        
        # Create trajectory setpoint with MIXED CONTROL
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # XY: Use velocity control (from Nav2, transformed to world frame)
        trajectory_msg.velocity[0] = float(vx_ned)   # North
        trajectory_msg.velocity[1] = float(vy_ned)   # East
        trajectory_msg.velocity[2] = 0.0  # No vertical velocity
        
        # Z: Use position control (altitude hold)
        trajectory_msg.position[0] = float('nan')  # Don't control X position
        trajectory_msg.position[1] = float('nan')  # Don't control Y position
        if self.hover_altitude_ned is not None:
            trajectory_msg.position[2] = float(self.hover_altitude_ned)  # Hold current altitude
        else:
            trajectory_msg.position[2] = float('nan')
        
        # Yaw rate (same convention)
        trajectory_msg.yaw = float('nan')  # Let PX4 control yaw based on yawspeed
        trajectory_msg.yawspeed = float(msg.angular.z * self.yaw_rate_scale)
        
        # Publish
        self.trajectory_setpoint_pub.publish(trajectory_msg)
        
        # Log for debugging every 50 messages
        if not hasattr(self, '_msg_count'):
            self._msg_count = 0
        self._msg_count += 1
        
        if self._msg_count % 50 == 0:
            self.get_logger().info(
                f'CMD: body[vx={vx_body:.2f}, vy={vy_body:.2f}] -> '
                f'NED[N={vx_ned:.2f}, E={vy_ned:.2f}] | yaw={np.degrees(self.current_yaw):.1f}°'
            )
    
    def timer_callback(self):
        """Publish offboard control mode at regular intervals"""
        if not self.enable_bridge:
            return
        
        # Don't interfere until we've received at least one Nav2 command
        if not self.first_cmd_received:
            return
        
        # Check for command timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # Create offboard control mode message
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if time_since_cmd < self.cmd_timeout:
            # Active control - velocity mode
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
        else:
            # Timeout - ALTITUDE HOLD MODE
            # Use position control for Z, velocity for XY
            offboard_msg.position = True   # Enable position control
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            
            # Send altitude hold setpoint
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Zero horizontal velocity
            trajectory_msg.velocity[0] = 0.0
            trajectory_msg.velocity[1] = 0.0
            trajectory_msg.velocity[2] = 0.0
            
            # Hold altitude using position control
            if self.hover_altitude_ned is not None:
                trajectory_msg.position[0] = float('nan')  # Don't control X position
                trajectory_msg.position[1] = float('nan')  # Don't control Y position
                trajectory_msg.position[2] = float(self.hover_altitude_ned)  # Hold altitude in NED
            else:
                # Fallback: if we don't have altitude yet, use current
                if self.current_altitude_ned is not None:
                    trajectory_msg.position[0] = float('nan')
                    trajectory_msg.position[1] = float('nan')
                    trajectory_msg.position[2] = float(self.current_altitude_ned)
                else:
                    # Last resort: all NaN (let PX4 handle it)
                    trajectory_msg.position[0] = float('nan')
                    trajectory_msg.position[1] = float('nan')
                    trajectory_msg.position[2] = float('nan')
            
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = 0.0
            
            self.trajectory_setpoint_pub.publish(trajectory_msg)
            
            # Log once when entering hover mode
            if not hasattr(self, '_hover_logged') or time_since_cmd < self.cmd_timeout + 0.2:
                self._hover_logged = True
                alt_m = -self.hover_altitude_ned if self.hover_altitude_ned is not None else 0.0
                self.get_logger().info(f'Nav2 timeout - ALTITUDE HOLD active at {alt_m:.2f}m')
        
        self.offboard_control_mode_pub.publish(offboard_msg)
        
        # Periodic status log
        if int(time_since_cmd * 10) % 50 == 0:  # Every 5 seconds
            status = "OFFBOARD" if self.offboard_mode else "NOT OFFBOARD"
            mode = "ALTITUDE HOLD" if time_since_cmd >= self.cmd_timeout else "NAV2 ACTIVE"
            alt_m = -self.hover_altitude_ned if self.hover_altitude_ned is not None else 0.0
            self.get_logger().info(
                f'Bridge: {status} | {mode} | '
                f'Time since cmd: {time_since_cmd:.1f}s | '
                f'Hover alt: {alt_m:.2f}m | Yaw: {np.degrees(self.current_yaw):.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()