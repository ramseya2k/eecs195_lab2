#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg improt Twist
from geometry_msgs.msg import PoseStamped
from rrt import find_path_RRT 

import tf
import random
import numpy as np
import math
from math import sqrt, sin, cos, atan2, ceil floor


def map_update(information): # references
	global current_map
	global current_resolution
	global current_origin
	global current_data
	current_map = [information.info.height][information.info.width] # height & width
	current_resolution = information.info.resolution
	current_origin = information.info.origin
	current_data = information.data


def get_index_from_coordinates():


rospy.init_node('RRT_NODE', anonymouse=False)

# publisher 
publish = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)

# subscribers 
rospy.Subscriber('/map', OccupancyGrid, map_update)
rospy.Subscriber('/start_goal', Float64MultiArray 
