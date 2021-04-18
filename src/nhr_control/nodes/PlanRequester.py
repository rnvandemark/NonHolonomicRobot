#!/usr/bin/env python

import rospy
from nhr_msgs.msg import PlanRequest, Path
from geometry_msgs.msg import Pose2D
from sys import argv as sargv
from time import sleep

ROS_NODE_NAME = "nhr_plan_requester"

def handle_path(msg):
    print " => ".join("({0},{1},{2})".format(p.position.y,p.position.x,p.position.theta) for p in msg.backtrack_path)
    move_commands = [p.move_cmd for p in reversed(msg.backtrack_path) if p.has_move_cmd]
    for cmd in move_commands:
        print "L:{0},R:{1}".format(cmd.left_wheel_speed, cmd.right_wheel_speed)
        # Publish command with given wheel speeds
        rospy.sleep(cmd.time_elapsed)
    # Publish 0 wheel speed command
    rospy.signal_shutdown("Received path, ready for clean shutdown.")

def main():
    # Capture required user input
    assert(len(sargv) == 5)

    ii = None; ij = None; fi = None; fj = None
    try:
        ii = float(sargv[1])
        ij = float(sargv[2])
        fi = float(sargv[3])
        fj = float(sargv[4])
    except:
        print "Inputs must be numbers."
        return

    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    path_pub = rospy.Publisher(
        "/nhr/plan_request",
        PlanRequest,
        queue_size=1
    )
    path_sub = rospy.Subscriber(
        "/nhr/path",
        Path,
        handle_path,
        queue_size=1
    )

    sleep_time_s = 0.5
    print "First sleeping for {0} second(s)...".format(sleep_time_s)
    sleep(sleep_time_s)
    path_pub.publish(PlanRequest(
        init_position=Pose2D(x=ii, y=ij, theta=0),
        final_position=Pose2D(x=fi, y=fj, theta=0)
    ))
    print "Plan request published."
    rospy.spin()

if __name__ == "__main__":
    main()
