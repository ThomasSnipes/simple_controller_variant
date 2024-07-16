import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import NatNetClient as NatNet

class OptitrackNode(Node):
    def __init__(self):
        super().__init__('optitrack_node')
        self.publisher = self.create_publisher(PointStamped, 'optitrack/markers', 10)
        self.client = NatNet.NatNetClient()
        self.client.new_frame_listener = self.receive_new_frame
        self.client.rigid_body_listener = self.receive_rigid_body_frame
        self.client.run()

    def receive_new_frame(self, data_dict):
        pass

    def receive_rigid_body_frame(self, id, position, rotation):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = position[0]
        msg.point.y = position[1]
        msg.point.z = position[2]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published marker position: {position}")

    def shutdown(self):
        self.client.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OptitrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
