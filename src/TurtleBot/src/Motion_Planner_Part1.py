#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from math import sqrt
from geometry_msgs.msg import PoseStamped

# THIS IS FOR PART 1 OF THE ASSIGNMENT 
motionArray = [] # this will be used inside this file
def send_info(): # sends information & publishes it 
	global motionArray
	x = float(raw_input("Enter X position:\n"))
	y = float(raw_input("Enter Y position:\n"))
	theta = float(raw_input("Enter theta:\n"))
	mode = float(raw_input("Enter mode 0/1:\n"))
	motionArray = [x, y, theta, mode]	
	
	
	return Float64MultiArray(data=motionArray)
		
def main():
	global prompt_flag
	rospy.init_node('Motion_Planner', anonymous=False)
	pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
	rate = rospy.Rate(10)
	prompt_flag = True 
	while not rospy.is_shutdown():
		if prompt_flag:
			pub.publish(send_info())
			prompt_flag = False # reset flag after input is received
			rospy.sleep(.1)
	rate.sleep()
	

def pose_update(msg):
	global position_x
	global position_y
	global rotation
	global motionArray
	global prompt_flag
	# receive information from /gazebo/model_states about current position
	position_x = msg.pose[1].position.x
	position_y = msg.pose[1].position.y
	rotation = msg.pose[1].orientation.z
	if motionArray:
		goal_x = motionArray[0]
		goal_y = motionArray[1]
		goal_theta = motionArray[2]
		error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
		print(error)
		if motionArray[3] == 0:
			if(goal_x != 0) and (error <= .15):
				print("goal reached") 
				prompt_flag = True
				motionArray = []
		else:
			if(goal_x != 0) and (error < 0.05): # break to exit out of the loop and ask for input
				print("goal reached")
				prompt_flag = True
				motionArray = [] # reset to obtain new goal

if __name__ == '__main__':
	try:
		rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update)
		main()
	except rospy.ROSInterruptException:
		pass
