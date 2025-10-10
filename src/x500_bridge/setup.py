from setuptools import find_packages, setup

package_name = 'x500_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_with_px4.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imamalghifary',
    maintainer_email='imamalghifary@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'odom_bridge = x500_bridge.odom_bridge:main',
        ],
    },
)
