#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


def get_goal_position():
	x = float(input("Enter the X coordinate of the goal:\n"))
	y = float(input("Enter the Y coordinate of the goal:\n"))
	return x, y

if __name__ == '__main__':
	rospy.init_node("ask_target", anonymous=False)
	pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		x, y = get_goal_position()
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = x 
		pose.pose.position.y = y 
		pose.pose.position.z = 0
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		pub.publish(pose)
		rate.sleep()
