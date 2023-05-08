#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import rrt 
from nav_msgs.msg import OccupancyGrid
import cv2

import tf
import random
import numpy as np
import math
from math import sqrt, sin, cos, atan2, ceil, floor


def map_update(information): # references
	global current_map
	global current_resolution
	global current_origin
	global current_data
	
	current_map = [[0 for j in range(information.info.height)] for i in range(information.info.width)]
	current_resolution = information.info.resolution
	current_origin = information.info.origin
	current_data = information.data
	
# for start_goal	
def start_goal_callback(msg):
	global current_map
	while not rospy.is_shutdown():
		x_start_real = msg.data[0]
		y_start_real = msg.data[1]
	
		x_goal_real = msg.data[2]
		y_goal_real = msg.data[3]
	
		x_start_index, y_start_index = get_index_from_coordinates(x_start_real, y_start_real)
		x_goal_index, y_goal_index = get_index_from_coordinates(x_goal_real, y_goal_real)
		start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
		path = rrt.find_path_RRT(start, goal, cv2.cvtColor(rrt.map_img(current_map), cv2.COLOR_GRAY2BGR)[::-1])
		pub.publish(path)
		rospy.loginfo(path)
	
	
	
	


def get_index_from_coordinates(x, y):
	global current_origin
	global current_resolution
	x = map_origin.position.x + int(round(x/current_resolution))
	y = map_origin.position.y + int(round(y/current_resolution))
	return x, y
	
	
def main():
	rospy.init_node('RRT_NODE', anonymous=False)
	# publisher 
	pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# subscribers 
		rospy.Subscriber('/map', OccupancyGrid, map_update)
		rospy.Subscriber('/start_goal', Float64MultiArray, start_goal_callback)
		rate.sleep()
		

if __name__ == '__main__':

	
	try:
		print("Running...\n")
		main()

	except rospy.ROSInterruptException:
		pass
