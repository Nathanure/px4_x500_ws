import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='ROS2 PX4 Interface Velocity Control with Collision Prevention',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard.offboard_control:main',
            'velocity_control = px4_offboard.velocity_control:main',
            'control = px4_offboard.control:main',
            'processes = px4_offboard.processes:main',
            # 'collision_prevention = px4_offboard.collision_prevention:main',
            'lidar_bridge = px4_offboard.obstacle_distance:main',
            'tf_publisher = px4_offboard.tf_publisher:main',
            'odom_publisher = px4_offboard.odom_publisher:main',
        ],
    },
)

