# import rclpy
# import yaml
# from rclpy.node import Node
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from geometry_msgs.msg import PointStamped
# from transforms3d.euler import quat2euler
# from custom_robot_msgs.msg import RobotCommandMsgs
# from .pid_controller import TargetPIDController
# from .pid_controller_node_sub import RobotControllerNode
# from tf2_ros import TransformBroadcaster, TransformStamped


# euler_from_quaternion = lambda q, axes='sxyz': quat2euler((q[3], q[0], q[1], q[2]), axes)

# class TargetPIDControllerNode(Node):
#     def __init__(self):
#         super().__init__('pid_controller')
#         #self.robot_controller_node = RobotControllerNode()
#         # Declare parameters
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('rate', 1.0),
#                 ('parent_frame', 'world'),
#                 ('child_frame', 'vehicle'),
#                 ('lower_velocity_threshold', 1.34),
#                 ('upper_velocity_threshold', 5.36),
#                 ('max_linear_velocity', 1.0),
#                 ('max_steering_angle_deg', 60.0),
#                 ('speed_gains', [0.1, 0.0, 0.0]),
#                 ('steering_gains', [0.5, 0.0, 0.1]),
#                 ('d_epsilon', 0.05),
#                 ('angle_epsilon', 8),
#                 ('num_agents', 1),
#                 ('tracked_rigid_bodies', '{}')
#             ])
        
        
#         # Read parameters
#         self.rate = self.get_parameter('rate').value
#         self.parent_frame = self.get_parameter('parent_frame').value
#         self.child_frame = self.get_parameter('child_frame').value
#         lower_threshold_v = self.get_parameter('lower_velocity_threshold').value
#         upper_threshold_v = self.get_parameter('upper_velocity_threshold').value
#         max_linear_velocity = self.get_parameter('max_linear_velocity').value
#         max_steering_angle_deg = self.get_parameter('max_steering_angle_deg').value
#         speed_gains = self.get_parameter('speed_gains').value
#         steering_gains = self.get_parameter('steering_gains').value
#         d_epsilon = self.get_parameter('d_epsilon').value
#         angle_epsilon = self.get_parameter('angle_epsilon').value
#         self.num_agents = self.get_parameter('num_agents').value

#         self.period = 1.0 / self.rate

#         self.target = None
#         self.controller = TargetPIDController(
#             max_linear_velocity=max_linear_velocity,
#             max_steering_angle=max_steering_angle_deg,
#             speed_gains=speed_gains,
#             steering_gains=steering_gains,
#             d_epsilon=d_epsilon,
#             angle_epsilon=angle_epsilon
#         )

#         self.controllers = {}
#         # Create subscribers
#         #self.target_sub = self.create_subscription(PointStamped, 'custom_topic', self.set_target, 10)
#         #self.target_sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
#         for i in range(self.num_agents):
#             self.controllers[f'robot_{i}'] = RobotControllerNode()
#             self.create_subscription(PointStamped, f'/robot_{i}/robot_coordinates', lambda msg, i=i: self.set_target(msg, i), 10)
        
#         # Create transform listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Create timers
#         self.timer = self.create_timer(self.period, self.control_loop)

#         self.targets = {} 

#         self.robot_publishers = {}
#         self.create_publishers()

#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         self.get_logger().info('Node started!')

    # def handle_rigid_bodies(self, msg):
    #     # Assuming msg is of type RigidBody
    #     for body in msg.rigidbodies:
    #         name = body.rigid_body_name
    #         if name in self.tracked_rigid_bodies:  # Adjust the name as per your Optitrack setup
    #             x = body.pose.position.x
    #             y = body.pose.position.y
    #             z = body.pose.position.z
    #             # Use x, y, z in your application logic (e.g., setting targets for robots)
    #             self.set_target(x, y, z)  # Example function to set robot target

    #             # Use the pose to set the target
    #             if name == 'target_body_name':  # Replace with the actual target body name
    #                 self.set_target(body.pose.position.x, body.pose.position.y)


#     # def set_target(self, x, y):
#     #     self.target = (x, y)
#     #     self.controller.set_target((x, y))

#     def set_target(self, msg, robot_id):
#         self.get_logger().info(f"Received target for robot_{robot_id}: {msg}")
#         self.controllers[f'robot_{robot_id}'].set_target((msg.point.x, msg.point.y))

#     def create_publishers(self):
#         for i in range(self.num_agents):  
#             topic_name = f'/robot_{i}/robot_commands'
#             self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)


#     def get_publisher(self, topic_name):
#         if topic_name not in self.robot_publishers:
#             self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
#         return self.robot_publishers[topic_name]


#     def get_vehicle_pose(self):
#         try:
#             trans = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())
#         except TransformException as ex:
#             self.get_logger().info(f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}')
#             return None
#         quaternion_message = trans.transform.rotation
#         quaternion = (quaternion_message.x, quaternion_message.y, quaternion_message.z, quaternion_message.w)
#         _, _, orientation = euler_from_quaternion(quaternion)
#         return (trans.transform.translation.x, trans.transform.translation.y, orientation)


#     def set_target(self, msg):
#         self.get_logger().info(f"Received target: {msg}")
#         self.target = msg
#         self.controller.set_target((msg.point.x, msg.point.y))
   

    # def control_loop(self):
    #     for i in range(self.num_agents):
    #         msg = RobotCommandMsgs()
    #         controller = self.controllers[f'robot_{i}']
            
    #         if controller.target is not None:
    #             pose = self.get_vehicle_pose(f'robot_{i}')
    #             if pose is not None:
    #                 speed, steering_angle_velocity = controller.compute_controls(pose)
                    
    #                 msg.linear_velocity = float(speed)
    #                 msg.angular_velocity = float(steering_angle_velocity)
    #                 msg.x, msg.y = controller.target[:2]
    #                 msg.z = 0.0  # Update if needed

    #                 self.get_publisher(f'/robot_{i}/robot_commands').publish(msg)
    #             else:
    #                 self.get_logger().info(f'Vehicle pose for robot_{i} is not available.')
    #                 msg.linear_velocity = 0.0
    #                 msg.angular_velocity = 0.0
    #         else:
    #             self.get_logger().info(f'Target for robot_{i} is not set.')
    #             msg.linear_velocity = 0.0
    #             msg.angular_velocity = 0.0

    #         self.get_logger().info(f"Publishing control message to robot_{i}: speed={msg.linear_velocity}, steering_angle_velocity={msg.angular_velocity}")

#     # def control_loop(self):
#     #     msg = RobotCommandMsgs()
        
#     #     if self.controller.target is not None:
#     #         pose = self.get_vehicle_pose()
#     #         if pose is not None:
#     #             speed, steering_angle_velocity = self.controller.compute_controls(pose)
#     #             #x, y, z = self.get_optitrack_coords()

#     #             msg.linear_velocity = float(speed)
#     #             msg.angular_velocity = float(steering_angle_velocity)
#     #             msg.x = pose[0]
#     #             msg.y = pose[1]

#     #             for i in range(self.num_agents): 
#     #                 self.get_logger().info(f"Publishing control message to robot_{i}: speed={msg.linear_velocity}, steering_angle_velocity={msg.angular_velocity}")
#     #                 self.get_publisher(f'/robot_{i}/robot_commands').publish(msg)
#     #         else:
#     #             self.get_logger().info('Vehicle pose is not available.')
#     #             msg.linear_velocity = 0.0
#     #             msg.angular_velocity = 0.0
#     #     else:
#     #         self.get_logger().info('Target is not set.')
#     #         msg.linear_velocity = 0.0
#     #         msg.angular_velocity = 0.0

# def main(args=None):
#     rclpy.init(args=args)
#     pid_controller_node = TargetPIDControllerNode()
#     rclpy.spin(pid_controller_node)
#     pid_controller_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from transforms3d.euler import quat2euler
from custom_robot_msgs.msg import RobotCommandMsgs
from mocap4r2_msgs.msg import RigidBodies
from .pid_controller import TargetPIDController
from .pid_controller_node_sub import RobotControllerNode
from tf2_ros import TransformBroadcaster, TransformStamped

euler_from_quaternion = lambda q, axes='sxyz': quat2euler((q[3], q[0], q[1], q[2]), axes)

class TargetPIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.robot_controller_node = RobotControllerNode()
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 1.0),
                ('parent_frame', 'world'),
                ('child_frame', 'vehicle'),
                ('lower_velocity_threshold', 1.34),
                ('upper_velocity_threshold', 5.36),
                ('max_linear_velocity', 1.0),
                ('max_steering_angle_deg', 60.0),
                ('speed_gains', [0.1, 0.0, 0.0]),
                ('steering_gains', [0.5, 0.0, 0.1]),
                ('d_epsilon', 0.05),
                ('angle_epsilon', 8),
                ('num_agents', 1),
                ('tracked_rigid_bodies', '{}')
            ])
        
        
        # Read parameters
        self.rate = self.get_parameter('rate').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        lower_threshold_v = self.get_parameter('lower_velocity_threshold').value
        upper_threshold_v = self.get_parameter('upper_velocity_threshold').value
        max_linear_velocity = self.get_parameter('max_linear_velocity').value
        max_steering_angle_deg = self.get_parameter('max_steering_angle_deg').value
        speed_gains = self.get_parameter('speed_gains').value
        steering_gains = self.get_parameter('steering_gains').value
        d_epsilon = self.get_parameter('d_epsilon').value
        angle_epsilon = self.get_parameter('angle_epsilon').value
        self.num_agents = self.get_parameter('num_agents').value

        self.period = 1.0 / self.rate

        self.target = None
        self.controller = TargetPIDController(
            max_linear_velocity=max_linear_velocity,
            max_steering_angle=max_steering_angle_deg,
            speed_gains=speed_gains,
            steering_gains=steering_gains,
            d_epsilon=d_epsilon,
            angle_epsilon=angle_epsilon
        )

        self.controllers = {}

        tracked_rigid_bodies_param = self.get_parameter('tracked_rigid_bodies').get_parameter_value().string_value
        
        self.tracked_rigid_bodies = yaml.safe_load(tracked_rigid_bodies_param)
        # Create subscribers
        #self.target_sub = self.create_subscription(PointStamped, 'custom_topic', self.set_target, 10)
        #self.target_sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)

        # Original pub & sub
        self.command_pub = self.create_publisher(RobotCommandMsgs, 'robot_commands', 10)
        #self.target_sub = self.create_subscription(PointStamped, 'custom_topic', self.set_target, 10)
        
        # Create subscribers
        # for i in range(self.num_agents):
        #     self.controllers[f'robot_{i}'] = RobotControllerNode()
        #     self.create_subscription(PointStamped, f'/robot_{i}/robot_coordinates', lambda msg, i=i: self.set_target(msg, i), 10)
        
        self.target_sub = self.create_subscription(PointStamped, 'custom_topic', self.set_target, 10)
        
        self.subscription = self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
        
        # Create transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timers
        self.timer = self.create_timer(self.period, self.control_loop)

        self.targets = {}
        self.robot_pos = {}
        # Initialize target variables
        self.target_x = None
        self.target_y = None 

        self.robot_publishers = {}
        #self.create_publishers()

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Node started!')


    def set_target(self, msg):
        self.get_logger().info(f"Received target: {msg}")
        self.target = msg
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        # The message here is the point we are trying to reach
        # It is an arbitrary point created in the launch file
        self.controller.set_target((msg.point.x, msg.point.y))
        

    def create_publishers(self):
        for i in range(self.num_agents):  # Adjust the range to match the number of robots
            topic_name = f'/robot_{i}/robot_commands'
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
    
    def handle_rigid_bodies(self, msg):

        # Assuming msg is of type RigidBody
        for body in msg.rigidbodies:
            name = body.rigid_body_name
            if name in self.tracked_rigid_bodies:  
    
                # Create a TransformStamped message
                pose = body.pose
                t = TransformStamped()
                
                # Fill in the TransformStamped fields
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = msg.header.frame_id
                t.child_frame_id = self.tracked_rigid_bodies[name]
                
                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z
                t.transform.rotation = pose.orientation

                x = pose.position.x
                y = pose.position.y
                #z = pose.position.z

                orientation = pose.orientation
                quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

                _, _, yaw = euler_from_quaternion(quaternion)
                
                # Broadcast the transform
                self.tf_broadcaster.sendTransform(t)

                self.poses[name] = {
                    'position': (x, y),
                    'orientation': yaw
                }

                # Use the pose to set the target
                # if name == 'target_body_name':  # Replace with the actual target body name
                #     self.set_target(body.pose.position.x, body.pose.position.y)

        
    def control_loop(self):
        msg = RobotCommandMsgs()
        id = 0
        if self.controller.target is not None:

            for robot_name, pose_data in self.robot_poses.items():
                
                position = pose_data['position']
                orientation = pose_data['orientation']
                x = position[0]
                y = position[1]
                orientation = orientation[0]
                pose = x, y, orientation

                if pose is not None:
                    speed, steering_angle_velocity = self.controller.compute_controls(pose)
                
                    msg.linear_velocity = float(speed)
                    msg.angular_velocity = float(steering_angle_velocity)
                    msg.x = x
                    msg.y = y
                    msg.orientation = orientation
                    msg.x_target = self.target_x 
                    msg.y_target = self.target_y
                    
                    self.get_logger().info(f"Publishing control message to {robot_name}: speed={msg.linear_velocity}, steering_angle_velocity={msg.angular_velocity}")
                    self.get_publisher(f'/robot_{id}/robot_commands').publish(msg)
                else:
                    self.get_logger().info('Vehicle pose is not available.')
                    msg.linear_velocity = 0.0
                    msg.angular_velocity = 0.0
                    
                id += 1
        else:
            self.get_logger().info('Target is not set.')
            msg.linear_velocity = 0.0
            msg.angular_velocity = 0.0


    

    def get_publisher(self, topic_name):
        if topic_name not in self.robot_publishers:
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
        return self.robot_publishers[topic_name]


def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = TargetPIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()