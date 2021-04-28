# NonHolonomicRobot
# Robert Vandemark & Alec Lahr
# https://github.com/rnvandemark/NonHolonomicRobot

// Build the catkin workspace with:
./ws_build.sh

// If you experience errors building the workspace, try:
./ws_build.sh --clean

// Launch ROS master, visualization, and gazebo
roslaunch nhr_sim main.launch clearance:=5
// clearance is an optional argument, defaults to 0

// Run path planner node
// After having executed the above launch file, use another terminal to request a path
// from start to finish with specified wheel speeds
rosrun nhr_control PlanRequester.py [start_x] [start_y] [goal_x] [goal_y] [low_speed] [high_speed]

// Here: start == (x=6.0,y=8.2), goal == (x=9.0,y=9.0), min,max wheel speeds == (6,10)
rosrun nhr_control PlanRequester.py 6 8.2 9 9 6 10
