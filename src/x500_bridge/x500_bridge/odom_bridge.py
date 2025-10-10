#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # Match PX4 QoS (BEST_EFFORT, KEEP_LAST)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber: PX4 vehicle odometry
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # Publisher: ROS 2 standard Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Odom bridge node started (PX4 → /odom + TF)")

    def odom_callback(self, msg: VehicleOdometry):
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])

        # Orientation (quaternion)
        odom.pose.pose.orientation.x = float(msg.q[0])
        odom.pose.pose.orientation.y = float(msg.q[1])
        odom.pose.pose.orientation.z = float(msg.q[2])
        odom.pose.pose.orientation.w = float(msg.q[3])

        # Linear velocity
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])

        # Angular velocity
        odom.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom.twist.twist.angular.z = float(msg.angular_velocity[2])

        # Publish Odometry
        self.odom_pub.publish(odom)

        # Broadcast TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()