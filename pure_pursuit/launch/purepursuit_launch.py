import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'pure_pursuit'

    config = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'purepursuit.yaml'
        )

    return LaunchDescription([
        Node(package=package_name,
             executable='path_tracker',
             name='purepursuit',
             parameters=[config],
             output='screen',
             # arguments=['--ros-args', '--log-level', 'debug']
        ),
    ])