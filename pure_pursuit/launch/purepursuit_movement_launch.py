import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    package_name = 'pure_pursuit'

    config = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'purepursuit.yaml'
        )

    purepursuit_node = Node(package=package_name,
                            executable='path_tracker',
                            name='purepursuit',
                            parameters=[config],
                            output='screen',
                            # arguments=['--ros-args', '--log-level', 'debug']
        )

    tf_update_node = Node(package=package_name,
                          executable='tf_broadcaster',
                          name='vehicle_broadcaster',
                          parameters=[config],
                          output='screen',
                          # arguments=['--ros-args', '--log-level', 'debug']
        )

    testpath_node = Node(package=package_name,
                         executable='testpath_pub',
                         name='planned_path',
                         output='screen',
                         # arguments=['--ros-args', '--log-level', 'debug']
        )

    reference_speed_exec = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 'topic', 'pub', '/reference_speed',
             'std_msgs/Float64', '"data: 5"'],
        shell=True
        )

    rviz_config = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'rviz',
            'purepursuit_rviz2.rviz'
        )

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     arguments=['-d', rviz_config]
        )

    return LaunchDescription([
        purepursuit_node,
        tf_update_node,
        reference_speed_exec,
        testpath_node,
        rviz_node,
    ])