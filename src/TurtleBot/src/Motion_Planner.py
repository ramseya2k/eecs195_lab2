#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from math import sqrt

arrayToPublish = [] # this will be used to publish information
motionArray = [] # this will be used inside this file
flag = True # this will be used to ask for user input first
def send_info(): # sends information & publishes it 
	global flag
	global motionArray
	if flag:
		x = float(raw_input("Enter X position:\n"))
		y = float(raw_input("Enter Y position:\n"))
		theta = float(raw_input("Enter theta:\n"))
		mode = float(raw_input("Enter mode 0/1:\n"))
		arrayToPublish = motionArray = [x, y, theta, mode]	
		arrayToPublish = Float64MultiArray(data=arrayToPublish)
		pub.publish(arrayToPublish)
		flag = False
	else:
		pass
	

def pose_update(msg):
	global flag
	global position_x
	global position_y
	global rotation
	global motionArray
	if flag:
		return
	else:
		# receive information from /gazebo/model_states about current position
		position_x = msg.pose[1].position.x
		position_y = msg.pose[1].position.y
		rotation = msg.pose[1].orientation.z

		goal_x = motionArray[0]
		goal_y = motionArray[1]
		goal_theta = motionArray[2]
		error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
		print(error)
		if(error > 0.05): # return false to indicate that the target has not reached its goal
			flag =  False
		if(goal_x != 0) and (error < 0.05): # return true to indicate the target has reached its goal
			flag = True

if __name__ == '__main__':
	rospy.init_node('Motion_Planner', anonymous=False)
	pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=5) # x, y, theta, mode 
	rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update)
	try:
		while not rospy.is_shutdown():
			if flag:
				send_info() # asks for user input 
				#flag = False # set the flag to false until next time robot reaches position
			else:
				continue # this will run pose_update(msg)
	except rospy.ROSInterruptException:
		pass
