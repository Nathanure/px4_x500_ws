from setuptools import setup
from glob import glob
import os

package_name = 'px4_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Maps directory (even if empty)
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*') if os.path.exists('maps') else []),
        # Worlds directory (even if empty)
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*') if os.path.exists('worlds') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathanca',
    maintainer_email='nathanca@todo.todo',
    description='Navigation package for PX4 offboard control',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = px4_navigation.cmd_vel_bridge:main',
        ],
    },
)