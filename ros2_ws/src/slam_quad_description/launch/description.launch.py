from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('slam_quad_description'),
        'urdf',
        'slam_quad.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ])
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    ])
