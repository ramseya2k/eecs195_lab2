#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from yaml import load
from std_msgs.msg import Header
import sys
import cv2
from cv_bridge import CvBridge

rospy.init_node('map_server', anonymous=False)
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

map_metadata = None

with open(sys.argv[1], 'r') as f:
	global map_metadata
	map_metadata = load(f)
	#print(map_metadata)
	
def publish_map():
	global map_metadata
	header = Header() # creation of header
	header.stamp = rospy.Time.now()
	header.frame_id = 'map'
	
	grid = OccupancyGrid()
	grid.header = header
	grid.info.resolution = map_metadata['resolution']
	grid.info.width = 384
	grid.info.height = 384
	grid.info.origin.position.x = map_metadata['origin'][0]
	grid.info.origin.position.y = map_metadata['origin'][1]
	grid.info.origin.position.z = map_metadata['origin'][2]
	grid.info.origin.orientation.x = 0.0
	grid.info.origin.orientation.y = 0.0
	grid.info.origin.orientation.z = 0.0
	grid.info.origin.orientation.w = 1.0
	
	image = cv2.imread('my_map.pgm', cv2.IMREAD_GRAYSCALE)
	
	occupancy_grid = []
	for row in image:
		row_data = [] #change this to something else then if occupancy grid of values doesn't work
		for pixel in row:
			if pixel == 0: # black
				occupancy_grid.append(100) # occupied space 
				raw_data.append(100) # occupied space
			elif pixel == 255: 
				occupancy_grid.append(0) # free space 
				raw_data.append(0) # free space 
			else:
				#occupancy_grid.append(-1) # unknown space
				raw_data.append(-1) # unknown space 
		occupancy_grid.append(row_data) # comment this out if any issues occur
	grid.data = [item for sublist in occupancy_grid for item in sublist] # flatten the grid
	
	# publish the occupancy grid message
	pub.publish(grid)
	
if __name__ == '__main__':
	try:
		publish_map()
	except rospy.ROSInterruptException:
		pass

