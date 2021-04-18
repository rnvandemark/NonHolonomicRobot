#!/usr/bin/env python

import rospy
from nhr_msgs.msg import PlanRequest, Path
from geometry_msgs.msg import Pose2D, Twist
from sys import argv as sargv
from math import sin, cos, pi as PI

ROS_NODE_NAME = "nhr_plan_requester"

# Get a ROS parameter from the server if it exists
def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value

# Helper function to convert from degrees to radians
def deg2rad(deg):
    return PI * deg / 180

# Calculate the linear velocity of the bot in x and y
def calc_lin_vel_yx(th, Ul, Ur, r):
    dd = (r/2) * (Ul+Ur)
    return dd*sin(th), dd*cos(th)

# Calculate the angular twist of the bot
def calc_ang_vel_z(Ul, Ur, r, L):
    return (r/L) * (Ur-Ul)

# Populate a twist message given motion of the wheels
def get_twist_from(th, Ul, Ur, r, L):
    twist = Twist()
    dy, dx = calc_lin_vel_yx(th, Ul, Ur, r)
    twist.linear.x = dx
    twist.linear.y = dy
    twist.angular.z = calc_ang_vel_z(Ul, Ur, r, L)
    return twist

# Publish a series of twist messages given a path to follow
def handle_path(msg, wheel_radius, lateral_separation, twist_pub):
    print " => ".join("({0},{1},{2})".format(p.position.y,p.position.x,p.position.theta) for p in msg.backtrack_path)
    move_commands = [(deg2rad(p.position.theta),p.move_cmd) for p in reversed(msg.backtrack_path) if p.has_move_cmd]
    for th,cmd in move_commands:
        twist_pub.publish(get_twist_from(
            th,
            cmd.left_wheel_speed,
            cmd.right_wheel_speed,
            wheel_radius,
            lateral_separation
        ))
        rospy.sleep(cmd.time_elapsed)
    twist_pub.publish(Twist())
    rospy.signal_shutdown("Received path, ready for clean shutdown.")

def main():
    # Capture required user input
    my_sargv = rospy.myargv(argv=sargv)
    assert(len(my_sargv) == 7)

    ii = None; ij = None; fi = None; fj = None; w1 = None; w2 = None
    try:
        ii = float(my_sargv[1])
        ij = float(my_sargv[2])
        fi = float(my_sargv[3])
        fj = float(my_sargv[4])
    except:
        print "Input coordinates must be numbers."
        return
    try:
        w1 = int(my_sargv[5])
        w2 = int(my_sargv[6])
    except:
        print "Input wheel speeds must be positive integers."
        return
    if (w1 <= 0) or (w2 <= 0):
        print "Input wheel speeds must be positive integers."
        return

    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    robot_r = get_rosparam("/nhr/robot_description/r")
    robot_L = get_rosparam("/nhr/robot_description/L")
    assert((robot_r != None) and (robot_L != None))
    path_pub = rospy.Publisher(
        "/nhr/plan_request",
        PlanRequest,
        queue_size=1
    )
    cmd_vel_pub = rospy.Publisher(
        "/cmd_vel",
        Twist,
        queue_size=1
    )
    path_sub = rospy.Subscriber(
        "/nhr/path",
        Path,
        lambda m: handle_path(m, robot_r, robot_L, cmd_vel_pub),
        queue_size=1
    )

    sleep_time_s = 0.5
    print "First sleeping for {0} second(s)...".format(sleep_time_s)
    rospy.sleep(sleep_time_s)
    path_pub.publish(PlanRequest(
        init_position=Pose2D(x=ii, y=ij, theta=0),
        final_position=Pose2D(x=fi, y=fj, theta=0),
        wheel_speed_minor=w1,
        wheel_speed_major=w2
    ))
    print "Plan request published."
    rospy.spin()

if __name__ == "__main__":
    main()
