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
        executable='connection_node',
        name='connection_node',
        output='screen',
    )

    nodes.append(connection_node)

    targets = []

    for i in range(num_agents):

        target_controller_node = Node(
            package=package_name,
            executable='robot_controller',
            name=f'robot_controller_{i}',
            namespace=f'robot_{i}',
            parameters=[config],
            output='screen',
            remappings=[
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

        # Half to 1st
        reference_location_exec = ExecuteProcess(
            cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/custom_topic',
                'geometry_msgs/PointStamped',
                '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
                            point: {x: -3312, y: -240, z: 0}}"'],
            shell=True
        )
        # First
        # reference_location_exec_2 = ExecuteProcess(
        #     cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/custom_topic',
        #         'geometry_msgs/PointStamped',
        #         '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
        #                     point: {x: -3312, y: -240, z: 0}}"'],
        #     shell=True
        # )


        # # Half to 2nd
        # reference_location_exec_3 = ExecuteProcess(
        #     cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/custom_topic',
        #         'geometry_msgs/PointStamped',
        #         '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
        #                     point: {x: -3312, y: -700, z: 0}}"'],
        #     shell=True
        # )
        # Second
        reference_location_exec_4 = ExecuteProcess(
            cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/custom_topic',
                'geometry_msgs/PointStamped',
                '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
                            point: {x: -3312, y: -1474, z: 0}}"'],
            shell=True
        )


        nodes.append(target_controller_node)
        nodes.append(tf_update_node)
        
        targets.append(reference_location_exec)
        # targets.append(reference_location_exec_2)
        # targets.append(reference_location_exec_3)
        targets.append(reference_location_exec_4)

        
    nodes.extend(targets)

    return LaunchDescription(nodes)