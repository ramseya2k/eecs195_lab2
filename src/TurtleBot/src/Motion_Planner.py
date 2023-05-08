#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from math import sqrt

# THIS IS FOR PART 2
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
	result = []
	for i in range(0, len(msg.data), 2):
		tmp = [msg.data[i], msg.data[i+1]]
		result.append(tmp)
	print(result)
	'''
	if msg.data[-1] == 1.0:
		rospy.loginfo("Robot has reached the target.\n")
		return
	'''

def main():
	rospy.init_node('Motion_Planner', anonymous=False)
	pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	
	while not rospy.is_shutdown():
		pub.publish(send_info()) # sends the start and goal coordinates to /start_goal
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)
			rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

'''
def check_if_done(msg):
	# subscribe to the trajectory, so what i was thinking was that maybe once the robot reaches its goal, check if the coordinates are equal to the x goal and y goal and if they are, then finish? or do error < 0.5 like the other PID controller
'''

'''
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
	rospy.init_node('Motion_Planner', anonymous=False)
	pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
	while not rospy.is_shutdown():
		pub.publish(send_info())
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rospy.Subscriber('/gazebo/Model_States', ModelStates, pose_update)
	

def pose_update(msg):
	global position_x
	global position_y
	global rotation
	global motionArray
	# receive information from /gazebo/model_states about current position
	position_x = msg.pose[1].position.x
	position_y = msg.pose[1].position.y
	rotation = msg.pose[1].orientation.z

	goal_x = motionArray[0]
	goal_y = motionArray[1]
	goal_theta = motionArray[2]
	error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
	print(error)
	if(goal_x != 0) and (error < 0.05): # break to exit out of the loop and ask for input
		break

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
'''
