# NonHolonomicRobot

// Launch ROS master and the path-planner node
roslaunch nhr_sim main.launch

// There are two optional arguments for main.launch:
// do_viz - run the visualizer node, which renders a video of any successfully generated path
// do_gazebo - run Gazebo and the turtlebot3 burger to execute the generated path

// For example, the following runs the visualizer and gazebo:
roslaunch nhr_sim main.launch do_viz:=1 do_gazebo:=1

// After having executed one of the above launch configurations, request a path
// from start to finish with specified wheel speeds
// Here: start == (x=6.0,y=8.2), goal == (x=9.0,y=9.0), min,max wheel speeds == (10,15)
rosrun nhr_control PlanRequester.py 6 8.2 9 9 10 15
