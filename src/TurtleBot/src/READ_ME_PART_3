Part 3:

You're going to want to open ~6 terminals for this.

1) roslaunch turtlebot3_gazebo turtlebo3_empty_world.launch
2) rosrun map_server map_server my_map.yaml 
- (IF IT DOES NOT WORK, SOURCE THE SETUP.BASH FOR THE ROS) 
- In this case for me: source /opt/ros/melodic/setup.bash 
3) rosrun TurtleBot RRT_Node.py
4) rosrun TurtleBot PID_Controller.py
5) rosrun TurtleBot Motion_Planner_Part3.py 
6) rosrun Turtlebot ask_target.py 


** THERE IS AN ISSUE WITH HOW THE ROBOT MOVES. 
** THE ROBOT WILL MOVE TO ALL THE POINTS IN THE TRAJECTORY, HOWEVER THE LOOP DOES NOT WORK, AND WILL REACH THE GOAL, AND WANDER OFF INTO SPACE.
** THE ROBOT DOES REACH THE GOALS INPUTTED, HOWEVER IT JUST WANDERS OFF ONCE IT DOES. YOU'LL HAVE TO RESTART EACH TIME. 
