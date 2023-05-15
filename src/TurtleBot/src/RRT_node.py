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

current_origin = None 
current_resolution = None
current_data = None
current_map = None

def map_update(information): # references
	global current_map
	global current_resolution
	global current_origin
	global current_data
	
	current_resolution = information.info.resolution
	current_origin = information.info.origin
	current_data = information.data
	current_map =  np.reshape(information.data, (information.info.width, information.info.height))#[[0 for j in range(information.info.height)] for i in range(information.info.width)]
	
# for start_goal	
def start_goal_callback(msg):
	global current_map
	#while not rospy.is_shutdown():
	x_start_real = msg.data[0]
	y_start_real = msg.data[1]
	
	x_goal_real = msg.data[2]
	y_goal_real = msg.data[3]
	
	x_start_index, y_start_index = get_index_from_coordinates(x_start_real, y_start_real)
	x_goal_index, y_goal_index = get_index_from_coordinates(x_goal_real, y_goal_real)
	start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
	path, graph = rrt.find_path_RRT(start, goal, current_map)
	#rospy.loginfo(path)
	result = []
	for i in range(len(path)):
		x, y = get_coordinates_from_index(path[i][0], path[i][1])
		result.append(x)
		result.append(y)
	pub.publish(Float64MultiArray(data=result))
	print("done")
	
	
	
	


def get_index_from_coordinates(x, y):
	global current_origin
	global current_resolution
	x = current_origin.position.x + int(round(x/current_resolution))
	y = current_origin.position.y + int(round(y/current_resolution))
	return x, y


def get_coordinates_from_index(x, y):
	global current_origin
	global current_resolution
	x = current_origin.position.x + (x + 0.5) * current_resolution
	y = current_origin.position.y + (y + 0.5) * current_resolution
	return x, y
if __name__ == '__main__':
	try:
		rospy.init_node('RRT_NODE', anonymous=False)
		#rate = rospy.Rate(10)
		# subscribers 
		rospy.Subscriber('/map', OccupancyGrid, map_update)
		rospy.Subscriber('/start_goal', Float64MultiArray, start_goal_callback)
		pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)		
		print("Running...\n")
		while not rospy.is_shutdown():
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
		pass
