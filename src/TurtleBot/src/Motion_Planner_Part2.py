#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from math import sqrt
from geometry_msgs.msg import PoseStamped

# THIS IS FOR PART 2
trajectory_printed = True 
def send_info(): # sends info & publishes
	x_start = float(raw_input("Enter start of X position:\n"))
	y_start = float(raw_input("Enter start of Y position:\n"))
	x_goal = float(raw_input("Enter goal of X position:\n"))
	y_goal = float(raw_input("Enter goal of Y position:\n"))
	arrayToPublish = [x_start, y_start, x_goal, y_goal]
	return Float64MultiArray(data=arrayToPublish)
	#arrayToPublish = Float64MultiArray(data=arrayToPublish)
	#pub.publish(arrayToPublish)

def trajectory_callback(msg):
	global trajectory_printed
	result = []
	for i in range(0, len(msg.data), 2):
		#tmp = [msg.data[i], msg.data[i+1]]
		result.append([msg.data[i], msg.data[i+1]])
	if result is not None:
		print(result)
		trajectory_printed = True 
		

def main():
	global trajectory_printed
	rospy.init_node('Motion_Planner', anonymous=False)
	pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)
	while not rospy.is_shutdown():
		if trajectory_printed: 
			pub.publish(send_info()) # sends the start and goal coordinates to /start_goal
			trajectory_printed = False # set it to false and wait for the next trajectory to print 
		rospy.sleep(0.1)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
