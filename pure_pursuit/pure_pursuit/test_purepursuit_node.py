#! /usr/bin/env python

'''
Test Pure Pusrsuit Node
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from .purepursuit import PurePursuit
from .test_paths import test_paths


class TestPurePursuitNode(Node):
    '''
    A class to represent a TestPurePursuitNode

    Attributes
    ----------
    node_name : str
        name of the node
    path_pub : rospy.publisher
        a variable called "command_pub" that holds an instance of the class
        rospy.Publisher
    timer : rospy.Timer
        a variable called "timer" that holds an instance of the class
        rospy.Timer

    Methods
    -------
    publish_path(event=None)
        Publishes the path coordinates for the vehcile to track/follow
    '''

    def __init__(self):
        '''
        Constructs all the necessary attributes for the TestPurePursuitNode
        object.

        Parameters
        ----------

        '''
        super().__init__('TestPurePursuitNode')
        # self.node_name = rospy.get_name()

        # Create publishers
        self.path_pub  = self.create_publisher(Path,'planned_path', 1)

        self.timer = self.create_timer(0.5, self.publish_path)

        self.get_logger().info('Node started!')

    def publish_path(self, event=None):
        '''
        Publishes the path coordinates for the vehcile to track/follow.

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Returns
        -------
        None
        '''
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "world"


        path_coords = test_paths[4]
	    # The list above is referencing from a list of tests that are in
        # a python folder in the test directory
        for x, y in path_coords:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "world"

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info('Published path!')


def main(args=None):
    # initialize node with rospy
    rclpy.init(args=args)
    # create the node object
    test_purepursuit = TestPurePursuitNode()
    # keep the node alive
    rclpy.spin(test_purepursuit)
    test_purepursuit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
