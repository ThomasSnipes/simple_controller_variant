import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    package_name = 'target_pid_controller'

    config = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'target_pid_controller.yaml'
        )
    
    nodes = []
    num_agents = 1

    connection_node = Node(
        package=package_name,
        executable='robot_controller',
        name='robot_controller',
        output='screen',
    )

    nodes.append(connection_node)

    for i in range(num_agents):
        target_controller_node = Node(
            package=package_name,
            executable='target_controller',
            name=f'target_controller_{i}',
            namespace=f'robot_{i}',
            parameters=[config],
            output='screen',
            remappings=[
                #('/planned_path', f'/robot_{i}/planned_path'),
                #('/robot_command', f'/robot_{i}/robot_commands'),
                #('/target', f'/robot_{i}/target'),
                ('/custom_topic', f'/robot_{i}/custom_topic'),
                ('/robot_commands', f'/robot_{i}/robot_commands'),
            ]
        )

        tf_update_node = Node(
            package=package_name,
            executable='tf_broadcaster',
            name=f'vehicle_broadcaster_{i}',
            namespace=f'robot_{i}',
            parameters=[config],
            output='screen',
        )

        reference_location_exec = ExecuteProcess(
            cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/custom_topic',
                'geometry_msgs/PointStamped',
                '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
                            point: {x: -2, y: 2, z: 0}}"'],
            shell=True
        )

        nodes.append(target_controller_node)
        nodes.append(tf_update_node)
        nodes.append(reference_location_exec)

    return LaunchDescription(nodes)