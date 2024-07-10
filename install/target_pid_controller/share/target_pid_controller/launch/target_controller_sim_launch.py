# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable

# def generate_launch_description():
#     package_name = 'target_pid_controller'

#     config = os.path.join(
#             get_package_share_directory(package_name),
#             'config',
#             'target_pid_controller.yaml'
#     )

#     # List to store all nodes to be launched
#     nodes = []
#     num_agents = 4
#     # Create instances of the target_controller_node and tf_update_node for each robot
#     for i in range(num_agents):
#         target_controller_node = Node(
#             package=package_name,
#             executable='target_controller',
#             name=f'target_controller_{i}',
#             namespace=f'robot_{i}',
#             parameters=[config],
#             output='screen',
#             remappings=[
#                 ('/planned_path', f'/robot_{i}/planned_path'),
#                 ('/speed_command', f'/robot_{i}/robot_commands'),
#                 ('/target', f'/robot_{i}/target'),
#             ]
#         )

#         tf_update_node = Node(
#             package=package_name,
#             executable='tf_broadcaster',
#             name=f'vehicle_broadcaster_{i}',
#             namespace=f'robot_{i}',
#             parameters=[config],
#             output='screen',
#         )

#         # Assign different target positions to each robot
#         # reference_location_exec = ExecuteProcess(
#         #     cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/target',
#         #          'geometry_msgs/PointStamped',
#         #          f'"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: world}}, point: {{x: {i+1 * 1/2}, y: {i+1 * 1/2}, z: 0}}}}"'],
#         #     shell=True
#         # )

#         reference_location_exec = ExecuteProcess(
#             cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/target',
#                 'geometry_msgs/PointStamped',
#                 '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
#                             point: {x: -2, y: 2, z: 0}}"'],
#             shell=True
#         )

       

#         # reference_location_exec = ExecuteProcess(
#         #     cmd=[FindExecutable(name='ros2'), 'topic', 'pub', f'/robot_{i}/target',
#         #          f'"{{"position": {{x: {i+1 * 1/2}, y: {i+1 * 1/2}, z: 0}}}}"'],
#         #     shell=True
#         # )


#         nodes.append(target_controller_node)
#         nodes.append(tf_update_node)
#         nodes.append(reference_location_exec)

#     rviz_config = os.path.join(
#             get_package_share_directory(package_name),
#             'config',
#             'rviz',
#             'target_controller_rviz2.rviz' 
#     )

#     rviz_node = Node(package='rviz2',
#                      executable='rviz2',
#                      name='rviz2',
#                      arguments=['-d'+str(rviz_config)]
#     )
    
#     nodes.append(rviz_node)

#     return LaunchDescription(nodes)



# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable


# def generate_launch_description():
#     package_name = 'target_pid_controller'

#     config = os.path.join(
#             get_package_share_directory(package_name),
#             'config',
#             'target_pid_controller.yaml'
#         )

#     target_controller_node = Node(package=package_name,
#                             executable='target_controller', #subscribed to /planned_path
#                             name='target_controller',
#                             parameters=[config],
#                             output='screen',
#                             # arguments=['--ros-args', '--log-level', 'debug']
#         )

#     tf_update_node = Node(package=package_name,
#                           executable='tf_broadcaster',
#                           name='vehicle_broadcaster',
#                           parameters=[config],
#                           output='screen',
#                           # arguments=['--ros-args', '--log-level', 'debug']
#         )

#     reference_location_exec = ExecuteProcess(
#         cmd=[FindExecutable(name='ros2'), 'topic', 'pub', '/target',
#              'geometry_msgs/PointStamped',
#              '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: world},\
#                          point: {x: -2, y: -4, z: 0}}"'],
#         shell=True
#         )
#     # reference_speed_exec = ExecuteProcess(
#     #     cmd=[FindExecutable(name='ros2'), 'topic', 'pub', '/reference_speed',
#     #          'std_msgs/Float64', '"data: 2"'],
#     #     shell=True
#     #     )

#     rviz_config = os.path.join(
#             get_package_share_directory(package_name),
#             'config',
#             'rviz',
#             'target_controller_rviz2.rviz' 
#         )

#     rviz_node = Node(package='rviz2',
#                      executable='rviz2',
#                      name='rviz2',
#                      #arguments=['-d', rviz_config]
#                      arguments=['-d'+str(rviz_config)]
                
#         )

#     return LaunchDescription([
#         target_controller_node,
#         tf_update_node,
#         reference_location_exec,
#         # testpath_node,
#         rviz_node,
#     ])

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

    # List to store all nodes to be launched
    nodes = []
    num_agents = 1
    # Create instances of the target_controller_node and tf_update_node for each robot
    for i in range(num_agents):
        target_controller_node = Node(
            package=package_name,
            executable='target_controller',
            name=f'target_controller_{i}',
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

    rviz_config = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'rviz',
            'target_controller_rviz2.rviz'
    )

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     arguments=['-d'+str(rviz_config)]
    )
    
    nodes.append(rviz_node)

    return LaunchDescription(nodes)
