#! /usr/bin/env python

import numpy as np
from numpy.linalg import norm


def pidController(kp, ki, kd):
    ierror = 0
    prev_error = 0
    error = 0

    while True:
        ierror += error
        derror = error - prev_error
        prev_error = error
        u = kp*error + ki*ierror + kd*derror
        print("U: ", u)
        error = (yield u)


class TargetPIDController:
    '''
    A class to calculate the orientation/speed to a target using a PID controller.

    Attribute
    ----------
    target : tuple
        location of the target as a 2-tuple (x, y)

    Methods
    -------
    set_target(target)
        Sets the target location.
    compute_controls()
        Computes linear velocity and steering angle to reach the target.
        
    '''

    # constructor
    def __init__(self, max_linear_velocity=1.0, max_steering_angle=60.0, speed_gains=None, 
        steering_gains=None, d_epsilon=0.05, angle_epsilon=8.0, target=None):
        '''
        Constructor for the TargetPIDController class. Sets values 
        for the PID controller, and no target is set.

        Parameters
        ----------
        max_linear_velocity : float
            ceiling for linear velocity
        max_rotation_velocity : float
            ceiling for rotation velocity
        target : tuple
            location of the target as a 2-tuple (x, y)

        Note
        ----
        The pose of the vehicle is a tuple of the form (x, y, theta).
        '''

        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = np.deg2rad(max_steering_angle) # converting to radians

        self.speed_gains = speed_gains
        self.steering_gains = steering_gains

        self.distance_pid = pidController(*self.speed_gains)
        next(self.distance_pid) # start the pid controller
        self.heading_pid = pidController(*self.steering_gains)
        next(self.heading_pid) # start the pid controller

        self.d_epsilon = d_epsilon # error bound around waypoint
        self.angle_epsilon = angle_epsilon # error bound for turning

        self.target = target

    def set_target(self, target):
        '''
        Converts/sets the target (tuple) into a numpy array.

        Parameters
        ----------
        target : tuple
            location of the target as a 2-tuple (x, y)

        Returns
        -------
        None
        '''
        self.target = np.array(target)

    def compute_controls(self, vehicle_pose):
        '''
        Given a set target, computes linear velocity and steering angle to
        reach target.

        Parameters
        ----------
        vehicle_pose : tuple
            pose of the vehicle as a 3-tuple (x, y, yaw)

        Returns
        -------
        tuple
            speed and steering angle calculated from the vehicle's pose
        '''
        if self.target is None:
            return 0.0, 0.0

        location = np.array(vehicle_pose[:2])
        orientation = vehicle_pose[2]

        dist = norm(self.target - location)
        if dist < self.d_epsilon: # if target was reached, switch to next one
            return 0.0, 0.0
        else:
            # compute heading of line of sight vector
            vector = self.target - location
            los_angle = np.arctan2(vector[1], vector[0])
            los_angle = np.rad2deg(los_angle)

            # compute error values for distance and heading
            error_dist = dist

            # error_heading = los_angle - orientation
            error_heading = los_angle - np.rad2deg(orientation)
            if error_heading > 180:
                error_heading = -(360 - error_heading)
            elif error_heading < -180:
                error_heading = 360 + error_heading

            # compute linear velocity and steering angle
            # use PID for linear velocity
            linear_velocity = self.distance_pid.send(error_dist)
            steering_angle = self.heading_pid.send(error_heading)
           
            #linear_velocity = float(min(linear_velocity, self.max_linear_velocity))
            linear_velocity = float(min(max(linear_velocity, 0.1), self.max_linear_velocity))
            steering_angle = float(max(min(np.deg2rad(steering_angle), self.max_steering_angle), -self.max_steering_angle))
            
            return (linear_velocity, steering_angle) # return computed linear velocity and steering angle as 2-tuple