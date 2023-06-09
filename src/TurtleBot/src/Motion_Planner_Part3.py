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

def current_position(msg): # from /gazebo/model_states 
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
	for i in range(len(trajectory)):
		traj_temp = trajectory[i]
		x = traj_temp[0]
		y = traj_temp[1]
		reference_pose_pub.publish(Float64MultiArray(data=[x, y, 0, 1])) #x, y, theta, mode
		rospy.loginfo("Moving to point ({}, {})".format(x, y))
		while sqrt((trajectory[i][0] - current_position_x)**2 + (trajectory[i][1] - current_position_y)**2) > 0.05:
			rospy.sleep(0.1)
			print("Distance: ", sqrt((trajectory[i][0] - current_position_x)**2 + (trajectory[i][1] - current_position_y)**2))
		print("Trajectory index: ", i)
	rospy.loginfo("Reached the goal!\n") 

if __name__ == '__main__':
	rospy.init_node('Motion_Planner', anonymous=False)
	rospy.loginfo("Ready!\n")
	start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	reference_pose_pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
	rospy.Subscriber('/gazebo/model_states', ModelStates, current_position) # obtain position of robot
	rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)
	#rospy.wait_for_message('/target_pose', PoseStamped) # waits for the first goal
	rospy.Subscriber('/target_pose', PoseStamped, goal_position_callback) # goal position
	monitor_robot_pose()
