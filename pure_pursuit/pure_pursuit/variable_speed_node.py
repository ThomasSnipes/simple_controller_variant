#! /usr/bin/env python

import math
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Float64


class VariableSpeedNode(Node):
    '''
    This node is meant for calculating a variable speed in order to
    show the relationship between lookahead and speed

    Attributes
    ----------
    reference_speed : rospy.Publisher
        the reference speed, Float64
    startTime : float
        the time the node is started

    Method
    ------
    sinusoidalSpeed():
        changes the speed sinusoidally (for testing)
    '''

    def __init__(self):
        '''
        Initializes by setting publisher and set the start time
        which will be used to calculate the speed
        '''
        super().__init__('variable_speed_node')

        # self.node_name = rospy.get_name() #TODO: needed?

        #set start time
        self.startTime = time.time()

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 1),
                ('speed_amplitude', 3),
                ('base_speed', 3.35),
            ])

        self.rate = self.get_parameter('rate').value

        # Create Publishers
        self.reference_speed_pub = self.create_publisher(Float64,
                                                         'reference_speed',
                                                         qos_profile=1)

        # Create timers
        self.timer = self.create_timer(1.0 / self.rate, self.sinusoidalSpeed)
        
        self.get_logger().debug('Node started!')


    def sinusoidalSpeed(self, event=None):
        '''
        Used the difference in time start and now to change the speed.
        The speed is centered at 3.35 which is the midpoint between
        the lookaway velocity bounds.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        amplitude = self.get_parameter('speed_amplitude').value
        base_speed = self.get_parameter('base_speed').value

        difTime = time.time() - self.startTime #difference in time
        thetaTime = difTime / 3 #divide delta by 60 (make more smooth)
        # calculate the speed
        speed = base_speed + amplitude * math.cos(thetaTime)
        speed_msg = Float64() # initialize the message
        speed_msg.data = speed # set msg data section
        self.reference_speed_pub.publish(speed_msg) # publish message

def main(args=None):
    rclpy.init(args=args)

    # create the node object
    speed_node = VariableSpeedNode()
    # keep the node alive
    rclpy.spin(speed_node)

    speed_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
