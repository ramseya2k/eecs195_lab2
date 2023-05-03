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


Kp_gain = 0.1 #1 
Ki_gain = 0.1
Kd_gain = 0 

position_x = 0
position_y = 0
rotation = 0

move_cmd = Twist()
cmd_vel = None


goal_x = 0
goal_y = 0
goal_angle = 0
mode = 0

def update_controller(msg):
	global goal_x
	global goal_y
	global goal_theta
	global mode
	goal_x = msg.data[0]
	goal_y = msg.data[1]
	goal_theta = msg.data[2]
	mode = msg.data[3]
	
	


def applyController():
	global goal_x
	global goal_y
	global goal_angle
	global Kp_gain
	global move_cmd
	global position_x
	global position_y
	global rotation
	global mode
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		if(mode == 0): # when mode is set to 0, activate the PID for the angular velocity first until robot faces ref point
			distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
			error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation)) # theta
			while(abs(error_angle) > .05): # activate the angular velocity until robot faces reference point
				move_cmd.angular.z = min(Kp_gain * error_angle, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)
				rate.sleep()
				distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
				error_angle_prev = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))

			error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
			while(error_distance > .1): # activate the linear velocity until robot gets to reference point
				move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)
				rate.sleep()
				error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))

			distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
			error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))			
			while(abs(error_angle) > .05): # activate the angular velocity again to turn the robot towards the final angle
				move_cmd.angular.z = min(Kp_gain * error_angle, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)
				rate.sleep()
				distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
				error_angle_prev = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))
		if(mode == 1): # activate both the angular and linear controller simulataneously 
			error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
			distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
	
			error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation)) # theta
			while(error_distance > 0.1):

	
				move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
				move_cmd.angular.z = min(Kp_gain * error_angle, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)

				rate.sleep()
				error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
				distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
	
				error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))
				

def pose_update(msg):
	global position_x
	global position_y
	global rotation
	global goal_x
	global goal_y
	global goal_angle


	#print(msg.pose[1])

	position_x = msg.pose[1].position.x
	position_y = msg.pose[1].position.y
	rotation = msg.pose[1].orientation.z


	error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
	#print(error)
	if (error > 0.05):
		applyController()
	if (goal_x != 0) and (error < 0.05):
		print("Target achieved")
		move_cmd.linear.x = 0
		move_cmd.angular.z = 0
		cmd_vel.publish(move_cmd)

if __name__ == '__main__':
	
	rospy.init_node('PID_Control_US', anonymous=False)
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5) # used to assign linear & angular velocities 
	rospy.Subscriber('/reference_pose', Float64MultiArray, update_controller) # x, y, theta, mode
	rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update) # contains information about current robot pose 

	rate = rospy.Rate(10)
	try:
		print("Running controller... please wait...")
		#time.sleep(1)
		while not rospy.is_shutdown():
			#rate.sleep(10)
			time.sleep(2)
	except:
		rospy.loginfo("Finished target.")
# this will be used for the input in motion planner 
'''
	print("Enter final x position")
	x_final = input()
	print("Enter final y position")
	y_final = input()
	print("Enter final angle position")
	angle_final = input()
	final = [x_final, y_final, angle_final]
	final_position = np.array(final)

	goal_x = final_position[0]
	goal_y = final_position[1]
	goal_angle = final_position[2]

	try:
		print("Running Controller...Please wait")
		#time.sleep(1)
		while not rospy.is_shutdown():
			rospy.sleep(0.1)
	except:
		rospy.loginfo("Finished target.")
'''







	









