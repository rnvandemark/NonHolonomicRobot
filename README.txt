# NonHolonomicRobot

Example to run system (execute each of these in a separate terminal):

// ROS master
roscore

// Create a planner, can handle multiple plan requests in its lifetime
// clearance := 0, minor wheel speed := 0
rosrun nhr_control Planner.py 0 50

// Have a node listen for paths planned to render a video of it (optional)
rosrun nhr_control Visualizer.py

// Request a path from start to finish
// start := (y=0.3,x=0.4), goal := (y=9.0,x=9.0)
rosrun nhr_control PlanRequester.py 0.3 0.4 9 9
