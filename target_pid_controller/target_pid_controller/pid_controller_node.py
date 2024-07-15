import math
import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from transforms3d.euler import quat2euler
from custom_robot_msgs.msg import RobotCommandMsgs
from visualization_msgs.msg import Marker
from .pid_controller import TargetPIDController
from .pid_controller_node_sub import RobotControllerNode

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
                ('num_agents', 1)
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

        # Create publishers
        #self.command_pub = self.create_publisher(RobotCommandMsgs, 'robot_commands', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Create subscribers
        self.target_sub = self.create_subscription(PointStamped, 'custom_topic', self.set_target, 10)

        # Create transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timers
        self.timer = self.create_timer(self.period, self.control_loop)

        self.robot_publishers = {}
        self.create_publishers()

        self.get_logger().info('Node started!')


    def get_publisher(self, topic_name):
        if topic_name not in self.robot_publishers:
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
        return self.robot_publishers[topic_name]
    

    def get_vehicle_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}')
            return None
        quaternion_message = trans.transform.rotation
        quaternion = (quaternion_message.x, quaternion_message.y, quaternion_message.z, quaternion_message.w)
        _, _, orientation = euler_from_quaternion(quaternion)
        return (trans.transform.translation.x, trans.transform.translation.y, orientation)

    def set_target(self, msg):
        self.get_logger().info(f"Received target: {msg}")
        self.target = msg
        self.controller.set_target((msg.point.x, msg.point.y))
        #msg.position = msg.point
        
        # Create and publish a Marker message
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def create_publishers(self):
        for i in range(self.num_agents):  # Adjust the range to match the number of robots
            topic_name = f'/robot_{i}/robot_commands'
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)

    def control_loop(self):
        msg = RobotCommandMsgs()
        
        if self.controller.target is not None:
            pose = self.get_vehicle_pose()
            if pose is not None:
                speed, steering_angle_velocity = self.controller.compute_controls(pose)
               
                msg.linear_velocity = float(speed)
                msg.angular_velocity = float(steering_angle_velocity)

                #for robot_id in range(len(self.robot_controller_node.IPs)):
                for i in range(self.num_agents): 
                    self.get_logger().info(f"Publishing control message to robot_{i}: speed={msg.linear_velocity}, steering_angle_velocity={msg.angular_velocity}")
                    self.get_publisher(f'/robot_{i}/robot_commands').publish(msg)
            else:
                self.get_logger().info('Vehicle pose is not available.')
                msg.linear_velocity = 0.0
                msg.angular_velocity = 0.0
        else:
            self.get_logger().info('Target is not set.')
            msg.linear_velocity = 0.0
            msg.angular_velocity = 0.0

def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = TargetPIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
