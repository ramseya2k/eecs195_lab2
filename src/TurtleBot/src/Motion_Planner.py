#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from math import sqrt
from geometry_msgs.msg import PoseStamped


goal_position = None # bound to be a global variable
trajectory = [] # initialized to be empty, global
current_position_x = 0.0
current_position_y = 0.0

def goal_position_callback(data): # from /target_pose
	# extract goal positon from the received message
	global goal_position, current_position_x, current_position_y 
	if goal_position is None:
		goal_position = data.pose.position
		rospy.loginfo("Received goal position: x={}, y={}".format(goal_position.x, goal_position.y))
		start_goal_pub.publish(Float64MultiArray(data=[current_position_x, current_position_y, goal_position.x, goal_position.y]))

	# send start and goal positions to RRT node 
	#if trajectory:
	#	start_goal_pub.publish(Float64MultiArray(data=[current_position_x, current_position_y, goal_position.x, goal_position.y])) 

def current_position_callback(msg): # from /gazebo/model_states 
	global current_position_x, current_position_y
	current_position_x = msg.pose[1].position.x
	current_position_y = msg.pose[1].position.y
	#rospy.loginfo("Current position: x={}, y={}".format(current_position_x, current_position_y))
		
def trajectory_callback(msg): # send the first point in the trajectory to the PID and publish to reference_pose topic
	# monitor the pose of the robot using using the /Gazebo/model_states topic 
	# once it gets very close to that position, the node should send the second position in the trajectory and wait until the robot is very close until it visits all the points
	global trajectory
	trajectory = []
	for i in range(0, len(msg.data), 2):
		x = msg.data[i]
		y = msg.data[i+1]
		trajectory.append([x, y])
	#rospy.loginfo(trajectory) 

def monitor_robot_pose():
	
	global trajectory, current_position_x, current_position_y
	while not trajectory: # waits until there is a trajectory 
		rospy.sleep(0.1)
	rospy.loginfo(trajectory) 
	for point in trajectory:
		x, y = point
		reference_pose_pub.publish(Float64MultiArray(data=[x, y, 0, 1])) #x, y, theta, mode
		rospy.loginfo("Moving to point ({}, {})".format(x, y))
		reached_point = False
		
		while not reached_point:
			distance = sqrt((trajectory[i][0] - current_position_x)**2 + (trajectory[i][1] - current_position_y)**2)
			rospy.sleep(0.1)
			print("Distance: ", distance)
			if distance < 0.05:
				reached_point = True
		rospy.loginfo("Reahced Point({}, {})".format(x, y))
		rospy.sleep(1) 
		
	rospy.loginfo("Reached the goal!\n") 

if __name__ == '__main__':
	rospy.init_node('Motion_Planner', anonymous=False)
	rospy.loginfo("Ready!\n")
	start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	reference_pose_pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
	rospy.Subscriber('/gazebo/model_states', ModelStates, current_position_callback) # obtain position of robot
	rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)
	
	#rospy.wait_for_message('/target_pose', PoseStamped) # waits for the first goal
	rospy.Subscriber('/target_pose', PoseStamped, goal_position_callback) # goal position
	monitor_robot_pose()

'''
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
'''
