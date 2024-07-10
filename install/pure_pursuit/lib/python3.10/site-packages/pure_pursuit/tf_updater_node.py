#! /usr/bin/env python

import math

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped

import numpy as np
from transforms3d.euler import euler2quat


quaternion_from_euler = lambda ai, aj, ak, axes='sxyz': \
                     tuple(euler2quat(ai, aj, ak, axes)[np.array([1, 2, 3, 0])])


class TFUpdaterNode(Node):
    '''
    The node simulates the vehicle motion based on the bycicle model, and
    Ackermann speed command messages.

    Attributes
    ----------
    current_command : AckermannDrive.ackermann_msgs.msg
        the current Ackermann,

    Methods
    -------
    getMessage(msg):
        gets the tf messages and stores them for the node
    updateVehicle(event=None):
        uses the message information to update the vehicle location
    '''

    def __init__(self):
        '''
        Initializes by setting up the publisher
        to change the tf and then also subscribe
        to the  the ackermann 'speed_command'
        '''
        super().__init__('tf_update_node')

        # self.node_name = rospy.get_name() #TODO: needed?

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 10),
                ('parent_frame', 'world'),
                ('child_frame', 'vehicle'),
            ])

        self.rate = self.get_parameter('rate').value
        self.period = 1.0 / self.rate
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # Create Subscriber
        self.command_sub = self.create_subscription(AckermannDrive,
                                                    'speed_command',
                                                    self.get_message,
                                                    qos_profile=10)

        # Create AckermannDrive Message holder
        self.ad_msg = AckermannDrive()

        #Iterables

        ##2D moventment
        self.x = 0
        self.y = 0

        # ##Quanterion
        # self.qx = 0
        # self.qy = 0
        # self.qz = 0
        # self.qw = 0

        ##Orientation(Angle)
        self.theta = 0

        #Create TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timers
        self.timer = self.create_timer(1.0 / self.rate,
                                       self.update_vehicle_location)
        self.get_logger().debug('Node started!')

    def get_message(self, msg):
        '''
        Get and set adMessage
        Parameters
            ----------
            msg : ackermannmsgs.msg.AckermannDrive
                car controll message
        Returns
        -------
        '''
        self.get_logger().info(f"Received AckermannDrive message: {msg}")

        self.ad_msg = msg

    def update_vehicle_location(self, event=None):
        '''
        Dependent on the Ackermann message it should update the location
        of the child frame vehicle. Since this method will be called
        in accordance to the rate, we will have the distance change
        be a function of the speed and steering angle

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Return
        ------
        None
        '''
        # get location current location
        delta_move = (1 / self.rate) * self.ad_msg.speed
        # use the period time the speed to have the total distance

        # self.theta = self.theta + self.ad_msg.steering_angle
        self.theta += self.ad_msg.steering_angle_velocity * self.period
        
        # The translation ins x is cosine of steering angle
        deltaX = delta_move * math.cos(self.theta)
        deltaY = delta_move * math.sin(self.theta)

        self.x = self.x + deltaX
        self.y = self.y + deltaY

        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi


        #transform part
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)

        # address qs
        t.transform.rotation.x = q[0]
        # self.qx = q[0]
        t.transform.rotation.y = q[1]
        # self.qy = q[1]
        t.transform.rotation.z = q[2]
        # self.qz = q[2]
        t.transform.rotation.w = q[3]
        # self.qw = q[3]

        #Sending Transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f"Updated vehicle location: ( {self.x:.2f} , {self.y:.2f} , {self.theta:.2f} )")

        # self.get_logger().info("( %5.2f , %5.2f , %5.2f )" %
        #                          (self.x, self.y, self.theta))


def main(args=None):
    rclpy.init(args=args)
    # create the node object
    tf_updater_node = TFUpdaterNode()
    # keep the node alive
    rclpy.spin(tf_updater_node)
    tf_updater_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
