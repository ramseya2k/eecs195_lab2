For Part 2:

1) Run roscore in a terminal
2) rosrun map_server map_server my_map.yaml 
- (IF IT DOES NOT WORK, SOURCE THE SETUP.BASH FOR THE ROS) 
- In this case for me: source /opt/ros/melodic/setup.bash 


3) rosrun Turtlebot RRT_Node.py 
4) rosrun Turtlebot Motion_Planner_Part2.py 
5) The motion planner should prompt for input, and then print out the trajectory coordinates, and prompt again. 
