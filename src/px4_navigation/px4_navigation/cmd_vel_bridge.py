#!/usr/bin/env python3
"""
CMD_VEL Bridge for PX4 Navigation
Bridges Nav2's /cmd_vel output to px4_offboard's /offboard_velocity_cmd input
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # Parameters
        self.declare_parameter('enable_bridge', True)
        self.declare_parameter('velocity_scale', 1.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.enable_bridge = self.get_parameter('enable_bridge').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # QoS profile matching px4_offboard
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to Nav2's cmd_vel output
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to px4_offboard's input topic
        self.offboard_vel_pub = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos_profile
        )
        
        # Statistics
        self.msg_count = 0
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('CMD_VEL Bridge started')
        self.get_logger().info(f'Enabled: {self.enable_bridge}')
        self.get_logger().info(f'Velocity Scale: {self.velocity_scale}')
        self.get_logger().info(f'Max Linear Vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max Angular Vel: {self.max_angular_vel} rad/s')
    
    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))
    
    def cmd_vel_callback(self, msg):
        """
        Receive cmd_vel from Nav2 and forward to px4_offboard
        
        Nav2 uses ROS convention:
        - linear.x: forward velocity
        - linear.y: lateral velocity (for holonomic robots)
        - angular.z: yaw rate
        
        px4_offboard expects the same format (it handles NED conversion internally)
        """
        if not self.enable_bridge:
            return
        
        # Create output message
        out_msg = Twist()
        
        # Apply scaling and clamping
        out_msg.linear.x = self.clamp(
            msg.linear.x * self.velocity_scale,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        
        out_msg.linear.y = self.clamp(
            msg.linear.y * self.velocity_scale,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        
        out_msg.linear.z = self.clamp(
            msg.linear.z * self.velocity_scale,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        
        out_msg.angular.z = self.clamp(
            msg.angular.z * self.velocity_scale,
            -self.max_angular_vel,
            self.max_angular_vel
        )
        
        # Publish to px4_offboard
        self.offboard_vel_pub.publish(out_msg)
        
        self.msg_count += 1
        
        # Log occasionally for debugging
        if self.msg_count % 100 == 0:
            self.get_logger().debug(
                f'Nav2 cmd: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.angular.z:.2f}] -> '
                f'PX4 cmd: [{out_msg.linear.x:.2f}, {out_msg.linear.y:.2f}, {out_msg.angular.z:.2f}]'
            )
    
    def print_stats(self):
        """Print bridge statistics"""
        if self.msg_count > 0:
            self.get_logger().info(f'Bridge active: {self.msg_count} messages forwarded')
            self.msg_count = 0


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