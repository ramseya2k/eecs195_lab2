#!/usr/bin/env python


import rospy 
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

import random
import numpy as np
import math


class RandomExplorer:
	def __init__(self):
		rospy.init_node('random_explorer', anonymous=False)
		rospy.Subscriber('/map', OccupancyGrid, self.map_update)
		rospy.Subscriber('/start_goal', Float64MultiArray, self.start_goal_callback)
		self.pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)
		
		
		self.current_map = None
		self.current_resolution = None
		self.current_origin = None
		self.current_data = None 
		self.rate = rospy.Rate(10)
		
	def map_update(self, information):
		self.current_resolution = information.info.resolution
		self.current_origin = information.info.origin
		self.current_data = information.data 
		self.current_map = np.reshape(information.data, (information.info.height, information.info.width))
		
	def get_index_from_coordinates(self, x, y):
		x_index = int(round((x - self.current_origin.position.y) / self.current_resolution))
		y_index = int(round((y - self.current_origin.position.x) / self.current_resolution))
		return x_index, y_index	
	
	def get_coordinates_from_index(self, x, y):
		x_real = (y * self.current_resolution) + self.current_origin.position.y
		y_real = (x * self.current_resolution) + self.current_origin.position.x
		return x_real, y_real
	
	def start_goal_callback(self, msg):
		x_start_real = msg.data[0]
		y_start_real = msg.data[1]
		x_goal_real = msg.data[2]
		y_goal_real = msg.data[3]
		
		x_start_index, y_start_index = self.get_index_from_coordinates(x_start_real, y_start_real)
		x_goal_index, y_goal_index = self.get_index_from_coordinates(x_goal_real, y_goal_real)
		start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
		path, graph = rrt.find_path_RRT(start, goal, self.current_map)
		result = []
		for i in range(len(path)):
			x, y = self.get_coordinates_from_index(path[i][0], path[i][1])
			result.append(x)
			result.append(y)
		self.pub.publish(Float64MultiArray(data=result))
		print("done")
		
	def explore_map(self):
		while not rospy.is_shutdown():
			random_x = random.uniform(self.current_origin.position.y, self.current_origin.position.y + (self.current_resolution * self.current_map_shape[0]))
			random_y = random.uniform(self.current_origin.position.x, self.current_origin.position.x + (self.current_resolution * self.current_map_shape[1]))
			x_goal_index, y_goal_index = self.get_index_from_coordinates(random_x, random_y)
			x_start_index, y_start_index = self.get_index_from_coordinates(random.uniform(self.current_origin.position.y, self.current_origin.position.y + (self_current_resolution * self.current_map.shape[0])), random.uniform(self.current_origin.position_x, + (self.current_resolution * self.current_map.shape[1]))) 
			start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
			path, graph = rrt.find_path_RRT(start,m goal, self.current_map)
			result = []
			for i in range(len(path)):
				x, y = self.get_coordinate_from_index(path[i][0], path[i][1]))
				result.append(x)
				result.append(y)
			self.pub.publish(Float64MultiArray(data=result))
			print("done")
			self.rate.sleep()


		
if __name__ == '__main__':
	try:
		explorer = RandomExplorer()
		explorer.explore_map()
	except rospy.ROSInterruptException:
		pass
