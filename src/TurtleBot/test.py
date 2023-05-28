#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid

from gazebo_msgs.msg import ModelStates

import tf
import random
import numpy as np
from math import sqrt, sin, cos, atan2, ceil, floor



scan = LaserScan()
omap = OccupancyGrid()
gtruth = ModelStates()


def update_scan(data):
	global scan
	scan = data

def update_omap(data):
	global omap
	omap = data

def update_ground_truth(data):
	global gtruth
	gtruth = data


def talker():
	algorithm_end = 0 
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.init_node('explore_map', anonymouse=False)
	rospy.Subscriber('/scan', LaserScan, update_scan)
	rospy.Subscriber('/map', OccupancyGrid, update_omap)
	rospy.Subscriber('/gazebo/model_states', ModelStates, update_ground_truth)
	rate = rospy.Rate(10)


	points_to_visit = []
	route = [] 
	points_visited = [] 
	car_name_index = 0

	for i in range(len(gtruth.name)):
		if gtruth.name[i] == rospy.get_param('TURTLEBOT3_MODEL') + "::base_footprint":
			print(gruth.name[i])
			car_name_index = i
			break

	while not rospy.is_shutdown():
		new_points_discovered = 0 
		car_x = gtruth.pose[car_name_index].position.x
		car_y = gtruth.pose[car_name_index].position.y 

		index_i = ceil((car_x - omap.info.origin.position.x) / omap.info.resolution)
		index_j = ceil((car_y - omap.info.origin.position.y) / omap.info.resolution)

		quat = [gtruth.pose[car_name_index].orientation.x, gtruth.pose[car_name_index].orientation.y, gtruth.pose[car_name_index].orientation.z, gtruth.pose[car_name_index].orientation.w]
		angle = tf.transformations.euler_from_quaternion(quat)[2]

		# keep a record of recent points 
		route.append((car_x, car_y))
		if (index_i, index_j) not in points_visited:
			points_visited.append((index_i, index_j))

		gaps = [] # search for gaps through lidar scans
		n = int(floor((scan.angle_max - scan.angle_min) / scan.angle_increment))
		for i in range(1, n):
			scan_angle = angle + scan.angle_min + i * scan.angle_increment
			gap_distance = scan.ranges[i] - scan.ranges[i-1] # gap between current and prev scan
			if(gap_distance > 0.5 and scan.ranges[i] != float('inf') and scan.ranges[i-1] != float('inf')):
				point_1 = (car_x + scan.ranges[i] * cos(scan_angle), car_y + scan.ranges[i] * sin(scan_angle))
				point_2 = (car_x + scan.ranges[i-1] * cos(scan_angle-scan.angle_increment), car_y + scan.ranges[i-1] * sin(scan_angle - scan.angle_increment))
				point_mid = ((point_1[0] + point_2[0]) / 2, (points_1[1] + points_2[1]) / 2, i)
				gaps.append(point_mid)


		while len(gaps) >= 2:
			gap_a = gaps.pop(0)
			gap_b = gaps[0]

			next_point = ((gap_a[0] + gap_b[0]) / 2, (gap_a[1] + gap_b[1]) / 2, int((gap_a[2] + gap_b[2])/ 2))
			i = ceil((next_point[0] - omap.info.origin.position.x) / omap.info.resolution)
			j = ceil((next_point[1] - omap.info.origin.position.y) / omap.info.resolution)

			next_index = (i, j)
			if next_index in points_visited:
				continue
			
			object_distance = scan.ranges[next_point[2]]
			point_distance = sqrt((next_point[0] - car_x) ** 2 + (next_point[1] - car_y) ** 2)

			if point_distance < object_distance + 0.05 and point_distance > 1:
				points_to_visit.append(next_point)
				new_points_discovered = new_points_discovered + 1 

			print("Points to visit: ", points_to_visit)
			print("Points visited: ", points_visited)


			if not points_to_visit:
				route.pop()
				temp = route.pop()
				poiints_to_visit.append(temp)


			if new_points_discovered == 0:
				points_to_visit.pop()

			target = points_to_visit[-1]

			K_x = 0.1
			K_z = 0.2

			error_angle = atan2((target[1] - car_y), (target[0] - car_x)) - car_angle
			error_position = sqrt((target[0] - car_x)**2 + (target[1] - car_y)**2)


			while abs(error_angle) > 0.05:
				print("target: ", target[0], target[1])
				print("position: ", car_x, car_y)
				print("error angle: ", error_angle)
				quat = [gtruth.pose[car_name_index].orientation.x, gtruth.pose[car_name_index].orientation.y, gtruth.pose[car_name_index].orientation.z, gtruth.pose[car_name_index].orientation.w]
				euler = tf.transformations.euler_from_quaternion(quat)
				car_angle = euler[2]
				error_angle = atan2((target[1] - car_y), (target[0] - car_x)) - car_angle
				move_cmd = Twist()
				move_cmd.angular.z = K_z * error_angle
				pub.publish(move_cmd)
				rospy.loginfo(move_cmd)
				rate.sleep()

			while error_position > 0.05:
				car_x = gtruth.pose[car_name_index].position.x
				car_y = gtruth.pose[car_name_index].position.y
			

				error_position = sqrt((target[0] - car_x)**2 + (target[1] - car_y)**2)
				print("target: ", target[0], target[1])
				print("position: ", car_x, car_y)

				move_cmd = Twist()
				move_cmd.linear.x = K_x * error_position
				pub.publish(move_cmd)
				rospy.loginfo(move_cmd)
				rate.sleep()

			if algorithm_end == 1:
				break
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

				
				
			
