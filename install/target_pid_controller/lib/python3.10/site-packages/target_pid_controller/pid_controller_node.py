import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
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
        # lower_threshold_v = self.get_parameter('lower_velocity_threshold').value
        # upper_threshold_v = self.get_parameter('upper_velocity_threshold').value
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
        self.poses = {}
        self.is_moving = False
        self.targets = []
        self.current_target_index = 0

        # Initialize target variables
        self.target_x = None
        self.target_y = None 

        self.robot_publishers = {}
        self.robot_subscribers = {}
        
        yaml_file = '/home/sdc/Desktop/mocap4ros2_ws/Archive/target_pid_controller/config/target_pid_controller_params.yaml'

        with open(yaml_file, 'r') as file:
            yaml_data = yaml.safe_load(file)

        self.tracked_rigid_bodies = yaml_data['target_controller']['ros__parameters']['tracked_rigid_bodies']
        self.list = []

        # for i in range(len(self.tracked_rigid_bodies)):
        #     self.list.append(self.tracked_rigid_bodies[i][0])
        #self.tracked_rigid_bodies = {int(k): v for k, v in self.tracked_rigid_bodies.items()}

        self.get_logger().info(f'Tracked Rigid Bodies: {self.tracked_rigid_bodies}')

        self.create_publishers()
        self.create_subscribers()
 
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
        
        # Prevent unused variable
        self.subscription

        # Create timers
        self.timer = self.create_timer(self.period, self.control_loop)

        self.get_logger().info('Node started now!')


    def create_publishers(self):
        for i in range(self.num_agents):  
            topic_name = f'/robot_{i}/robot_commands'
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
    

    def create_subscribers(self):
        for i in range(self.num_agents):  
            topic_name = f'/robot_{i}/custom_topic'
            self.robot_subscribers[topic_name] = self.create_subscription(PointStamped, topic_name, self.store_target, 10)
    

    def store_target(self, msg):
        self.get_logger().info(f"Received target: {msg}")

        if (msg.point.x, msg.point.y) not in self.targets:
            self.targets.append((msg.point.x, msg.point.y))
            self.get_logger().info(f"TARGETS VECTOR: {self.targets}")

            # If it's the first element, set it as initial target
            if len(self.targets) == 1:
                self.set_target(self.targets[0])


    def get_publisher(self, topic_name):
        if topic_name not in self.robot_publishers:
            self.robot_publishers[topic_name] = self.create_publisher(RobotCommandMsgs, topic_name, 10)
        return self.robot_publishers[topic_name]
    

    def set_target(self, target):
        self.target_x, self.target_y = target
        self.is_moving = True
        self.get_logger().info(f"Target set to: ({self.target_x}, {self.target_y})")
        self.controller.set_target(target)
    

    def switch_to_next_target(self):
        self.current_target_index += 1
        self.get_logger().info("switching")
        if self.current_target_index < len(self.targets):
            self.set_target(self.targets[self.current_target_index])
        else:
            self.get_logger().info("All targets have been reached")
            self.is_moving = False


    def is_target_reached(self, x, y):
        self.get_logger().info(f"TARGET X: {self.target_x}")
        self.get_logger().info(f"TARGET Y: {self.target_y}")
        return self.robot_controller_node.is_within_radius(x, y, self.target_x, self.target_y)


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

                orientation = pose.orientation
                quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

                _, _, yaw = euler_from_quaternion(quaternion)
                
                # Broadcast the transform
                self.tf_broadcaster.sendTransform(t)

                self.poses[name] = {
                    'position': (x, y),
                    'orientation': yaw
                }

        
    def control_loop(self):
        
        msg = RobotCommandMsgs()
        id = 0

        if self.is_moving and self.controller.target is not None:

            for robot_name, pose_data in self.poses.items():
                
                position = pose_data['position']
                orientation = pose_data['orientation']
                
                # Need to be multiplied to fit the Optitrack magnitude
                x = position[0] * 1000
                y = position[1] * 1000

                self.get_logger().info(f'X VALUE: {x}')
                self.get_logger().info(f'Y VALUE: {y}')
                #self.get_logger().info(f'ORIENTATION: {orientation}')
     
                pose = x, y, orientation

                if pose is not None:
                    speed, steering_angle, turn_angle = self.controller.compute_controls(pose)
                
                    msg.linear_velocity = float(speed)
                    msg.angular_velocity = float(steering_angle)
                    msg.x = x
                    msg.y = y
                    msg.orientation = orientation
                    msg.x_target = self.target_x 
                    msg.y_target = self.target_y
                    msg.angle = turn_angle
                    
                    self.get_logger().info(f"Publishing control message to {robot_name}: speed={msg.linear_velocity}, steering_angle_velocity={msg.angular_velocity}")
                    self.get_publisher(f'/robot_{id}/robot_commands').publish(msg)
                   
                    # Check if the robot has reached the current target
                    if self.is_target_reached(x, y):
                    #if speed == 0.0 and steering_angle == 0.0:
                        self.get_logger().info('TRUE')
                        # Check if all targets have been processed
                        if self.current_target_index < len(self.targets)-1:
                            self.switch_to_next_target()
                        else:
                            
                            msg.linear_velocity = 0.0
                            msg.angular_velocity = 0.0
                            self.get_publisher(f'/robot_{id}/robot_commands').publish(msg)
                            self.is_moving = False
                            return
                else:
                    self.get_logger().info('Vehicle pose is not available.')
                    msg.linear_velocity = 0.0
                    msg.angular_velocity = 0.0

                id += 1
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