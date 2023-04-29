#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64MultiArray
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transofmrations import euler_from_quaternion
import numpy as np
import os
import time 


Kp_gain = 1 
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

def applyController():
	global goal_x
	global goal_y
	global goal_angle
	global Kp_gain
	global move_cmd
	global position_x
	global position_y
	global rotation

	while not rospy.is_shutdown():
		if(mode == 1): 
			error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
			distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
	
			error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))
			while(error_distance > 0.1):

	
				move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
				move_cmd.angular.z = min(Kp_gain * error_angle, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)

				rate.sleep()
				error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
				distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
	
				error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))
		if(mode == 0):
			distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
	
			error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle-rotation))
			error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y-position_y, 2))
			while(error_distance > 0.1):
				move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
				cmd_vel.publish(move_cmd)
				rospy.loginfo(move_cmd)

				rate.sleep()

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
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	rospy.Subscriber('/reference_pose', Float64MultiArray, update_controller)
	rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update)

	r = rospy.Rate(10)

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








	









