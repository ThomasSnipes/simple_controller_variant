import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from mocap4ros2_msgs.msg import RigidBodies 
import yaml

class TargetController(Node):

    def __init__(self):
        super().__init__('target_setter')
        
        self.declare_parameter('tracked_rigid_bodies', '{}')
        tracked_rigid_bodies_param = self.get_parameter('tracked_rigid_bodies').get_parameter_value().string_value
        self.tracked_rigid_bodies = yaml.safe_load(tracked_rigid_bodies_param)
        self.get_logger().info(f'Tracked Rigid Bodies: {self.tracked_rigid_bodies}')

        self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
        
        self.target_publishers = {}

        for i in range(len(self.tracked_rigid_bodies)):
            topic_name = f'/robot_{i}/target'
            self.target_publishers[f'robot_{i}'] = self.create_publisher(PointStamped, topic_name, 10)
        
        self.targets = {}

    def handle_rigid_bodies(self, msg):
        for body in msg.rigidbodies:
            name = body.rigid_body_name
            if name in self.tracked_rigid_bodies:
                x = body.pose.position.x
                y = body.pose.position.y
                z = body.pose.position.z

                # Example function to set robot target
                self.set_robot_target(name, x, y, z)

    def set_robot_target(self, name, x, y, z):
        self.targets[name] = (x, y, z)
        
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        
        if name in self.target_publishers:
            self.get_logger().info(f"Publishing PointStamped message to {name}: {x}, {y}, {z}")
            self.target_publishers[name].publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
