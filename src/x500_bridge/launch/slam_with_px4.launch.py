from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to slam_toolbox launch
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_sync_launch.py"
    )

    return LaunchDescription([

        # Bridge Gazebo topics -> ROS 2
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # LIDAR → /scan
                "/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
                + "@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                # IMU
                "/world/walls/model/x500_lidar_2d_0/link/base_link/sensor/imu_sensor/imu"
                + "@sensor_msgs/msg/Imu@gz.msgs.IMU",
                # GPS
                "/world/walls/model/x500_lidar_2d_0/link/base_link/sensor/navsat_sensor/navsat"
                + "@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
                # Magnetometer
                "/world/walls/model/x500_lidar_2d_0/link/base_link/sensor/magnetometer_sensor/magnetometer"
                + "@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer",
                # Barometer
                "/world/walls/model/x500_lidar_2d_0/link/base_link/sensor/air_pressure_sensor/air_pressure"
                + "@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure",
                # Clock (simulation time sync)
                "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            ],
            remappings=[
                ("/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan", "/scan")
            ],
            output="screen",
            parameters=[{
                # Force best_effort QoS so ROS2 nodes (slam_toolbox) can actually subscribe
                'qos_overrides./scan.publisher.reliability': 'best_effort',
                'qos_overrides./world/walls/model/x500_lidar_2d_0/link/base_link/sensor/imu_sensor/imu.publisher.reliability': 'best_effort',
                'qos_overrides./clock.publisher.reliability': 'best_effort',
            }]
        ),

        # Custom node to convert PX4 /vehicle_odometry → /odom
        Node(
            package="x500_bridge",   # your odom_bridge node
            executable="odom_bridge",
            name="odom_bridge",
            output="screen",
        ),

        Node(
            package="x500_bridge",   # your odom_bridge node
            executable="fix_laser",
            name="fix_laser",
            output="screen",
        ),
        
        # Static TF: base_link → link (lidar mount)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "link"],
            name="lidar_tf_pub"
        ),

        # Static TF: map → odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            name="map_odom_tf_pub"
        ),

        # Slam Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                "use_sim_time": "true"
            }.items(),
        )
    ])