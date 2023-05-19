#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
robot_x = 0.0
robot_y = 0.0

def get_goal_position(): # asks for input 
	x = float(input("Enter the X coordinate of the goal:\n"))
	y = float(input("Enter the Y coordinate of the goal:\n"))
	return x, y

def distance_to_goal(x, y, goal_x, goal_y): # checks the distance between the current position and the goal 
	return ((goal_x - x)**2 + (goal_y-y)**2)**.5

def monitor_robot_pose(msg): # assigns the values to the current position of the robot 
	global robot_x, robot_y
	robot_x = msg.pose[1].position.x
	robot_y = msg.pose[1].position.y 

if __name__ == '__main__':
	rospy.init_node("ask_target", anonymous=False)
	pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
	rospy.Subscriber('/gazebo/model_states', ModelStates, monitor_robot_pose)
	rate = rospy.Rate(10)
	
	goal_x, goal_y = get_goal_position()

	while not rospy.is_shutdown():
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = goal_x # assigns the x goal 
		pose.pose.position.y = goal_y # assigns the y goal 
		pose.pose.position.z = 0
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		pub.publish(pose)
			 
		rospy.loginfo_throttle(1, "Distance to goal: {:.2f}".format(distance_to_goal(robot_x, robot_y, goal_x, goal_y)))
			 
		if distance_to_goal(robot_x, robot_y, goal_x, goal_y) < .1:
			 rospy.loginfo("Robot has reached the goal!\n")
			 goal_x, goal_y = get_goal_position()
		rate.sleep()
