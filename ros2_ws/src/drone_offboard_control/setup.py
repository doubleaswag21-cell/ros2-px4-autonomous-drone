from setuptools import find_packages, setup

package_name = 'drone_offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ammaar',
    maintainer_email='doubleaswag21@gmail.com',
    description='ROS 2 offboard control, SLAM-to-PX4 external vision integration, and GPS-denied autonomous waypoint navigation for PX4-based quadcopters',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_control = drone_offboard_control.offboard_control:main',
            'mapping_mission = drone_offboard_control.mapping_mission:main',
            'slam_to_px4 = drone_offboard_control.slam_to_px4:main',
            'lidar_time_converter = drone_offboard_control.lidar_time_converter:main',
            'gps_denied_waypoint = drone_offboard_control.gps_denied_waypoint:main',
        ],
    },
)
