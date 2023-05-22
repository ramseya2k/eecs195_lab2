#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64MultiArray
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import time

class Controller:
    def __init__(self):
        # Creates a node with name 'PID_Control_US' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('PID_Control_US', anonymous=False)

        # Publisher which will publish to the topic 'cmd_vell'.
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type ModelStates is received.
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)
        rospy.Subscriber('/reference_pose', Float64MultiArray, self.update_goals)
        # AGENT'S POSE in the gazebo environment
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0
        self.mode = None

        # Time discretization for the PID
        self.time_discr = 0.001 #.01

        # PID parameters for linear velocity
        self.error_prev_linear = 0
        self.integral_prev_linear = 0
        self.Kp_linear = 1
        self.Ki_linear = .1
        self.Kd_linear = 0

        # PID parameters for angular velocity
        self.error_prev_angular = 0
        self.integral_prev_angular = 0
        self.Kp_angular = 1
        self.Ki_gain_angular = .1
        self.Kd_gain_angular = 0

        self.rate = rospy.Rate(10)


    def update_pose(self, data):
        self.pose_x = round(data.pose[1].position.x, 4)
        self.pose_y = round(data.pose[1].position.y, 4)
        self.pose_theta = euler_from_quaternion([data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])[2]
	
    def update_goals(self, data):
        self.goal_x = round(data.data[0], 4)
        self.goal_y = round(data.data[1], 4)
        self.goal_theta = data.data[2]
        self.mode = data.data[3]
	

    def calc_distance(self):
        """Euclidean distance between current pose and the goal."""
        #return sqrt(pow((self.goal_x - self.pose_x), 2) + pow((self.goal_y - self.pose_y), 2)) #+ pow((self.goal_theta - self.pose_theta), 2))
        return ((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)**.5
    
    def angle_calculation(self):
        return atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)

    def proportional_linear(self):
        return min(self.Kp_linear * self.calc_distance(), 0.1)

    def derivative_linear(self):
        derivate_linear_vel = (self.calc_distance() - self.error_prev_linear) / self.time_discr
        updated_linear = self.Kd_linear * derivate_linear_vel
        self.error_prev_linear = self.calc_distance()  # update the error prior 
        return updated_linear

    def proportional_angular(self): 
        #error_angle = self.angle_calculation() - self.pose_theta
        return min(self.Kp_angular * (self.angle_calculation() - self.pose_theta), 0.1)

    def derivative_angular(self):
        error_angle = self.angle_calculation() - self.pose_theta
        derivative_angular = (error_angle - self.error_prev_angular) / self.time_discr
        updated_angular = self.Kd_gain_angular * derivative_angular
        # update the error prior
	self.error_prev_angular = error_angle 
        return updated_angular

    def integral_angular_vel(self):
        error_angle = self.angle_calculation() - self.pose_theta
        integral_angular_vel = self.integral_prev_angular +error_angle * self.time_discr
        updated_angular = self.Ki_gain_angular * integral_angular_vel
        # update the integral prior 
        self.integral_prev_angular = integral_angular_vel
        return updated_angular

    def PID_linear(self):
        pid = self.proportional_linear() + self.derivative_linear() + self.integral_vel()
        # set error priors for linear to 0 
        self.error_prev_linear = 0
        self.integral_prev_linear = 0
        return pid

    def PID_angular(self):
        pid_control_angular = self.proportional_angular() + self.derivative_angular() + self.integral_angular_vel()
        # update error priors for angular to 0
        self.error_prev_angular = 0
        self.integral_prev_angular = 0
        return pid_control_angular
 
    def integral_vel(self):
        integral_vel = self.integral_prev_linear + self.calc_distance() * self.time_discr
        updated_linear = self.Ki_linear * integral_vel
        self.integral_prev_linear = integral_vel
        return updated_linear

    def goal(self):
        while not rospy.is_shutdown():
                move_cmd = Twist()
		
		elif self.mode == 1:
			while self.calc_distance() >= 0.05:
				move_cmd.linear.x = self.PID_linear()
				move_cmd.angular.z = self.PID_angular()
				self.cmd_vel.publish(move_cmd)
				self.rate.sleep()
			move_cmd.linear.x = 0
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
			self.error_prev_linear = 0
			self.integral_prev_linear = 0
			self.error_prev_angular = 0
			self.integral_prev_angular = 0
			self.mode = None

        if self.mode == 0:
			while abs(self.angle_calculation() - self.pose_theta) >= 0.005:
				move_cmd.angular.z = self.PID_angular() # face the reference point 
				self.cmd_vel.publish(move_cmd)
				self.rate.sleep()
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
			
			while self.calc_distance() >= .15:
				move_cmd.linear.x = self.PID_linear() # go to the reference point 
				self.cmd_vel.publish(move_cmd)
				self.rate.sleep() 
				
			move_cmd.linear.x = 0
			self.cmd_vel.publish(move_cmd)
			
			while abs(self.angle_calculation() - self.pose_theta) >= 0.005:
				move_cmd.angular.z = self.PID_angular()
				self.cmd_vel.publish(move_cmd)
				self.rate.sleep()
				
			move_cmd.angular.z = 0 
			self.cmd_vel.publish(move_cmd)
			self.error_prev_linear = 0
			self.integral_prev_linear = 0
			self.error_prev_angular = 0
			self.integral_prev_angular = 0
			self.mode = None    		
		else:
		        rospy.loginfo("Waiting for mode to be 0 or 1 to continue.")
			while self.mode is None:
				self.rate.sleep()
				move_cmd.linear.x = 0
				move_cmd.angular.z = 0
				self.cmd_vel.publish(move_cmd)
if __name__ == '__main__':
    try:
        x = Controller()
        x.goal()
    except:
        rospy.ROSInterruptException
