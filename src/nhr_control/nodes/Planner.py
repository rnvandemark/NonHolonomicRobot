#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from nhr_msgs.msg import PlanRequest, NeighborsPose2D, BacktrackNode, MoveCommand, Path
import cv2
import numpy as np
from math import sqrt, cos, sin, pi as PI
from sys import stdout, argv as sargv
from gc import collect as gc_collect

ROS_NODE_NAME = "nhr_planner"

BOARD_H = 10
BOARD_W = 10
BOARD_O = 30
ROBOT_RADIUS = 0.153

GRID_D = 10
GRID_H = BOARD_H * GRID_D
GRID_W = BOARD_W * GRID_D
GRID_O = int(360 / BOARD_O)
GRID_ROBOT_RADIUS = ROBOT_RADIUS * GRID_D

# Board Obstacles
quads = [[2.5, 57.5, 2.5, 42.5, 17.5, 42.5, 17.5, 57.5],
         [37.5, 57.5, 37.5, 42.5, 62.5, 42.5, 62.5, 57.5],
         [72.5, 40, 72.5, 20, 87.5, 20, 87.5, 40]]
elips = [[20, 20, 10, 10],
         [20, 80, 10, 10]]

def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value

def printr(text):
    stdout.write("\r" + text)
    stdout.flush()

def quad_check(x0, y0, quad):
    # extract coords out of quad list
    x1 = quad[0]
    y1 = quad[1]
    x2 = quad[2]
    y2 = quad[3]
    x3 = quad[4]
    y3 = quad[5]
    x4 = quad[6]
    y4 = quad[7]

    # check if the point is within the restricted half-plane side of each line
    chk1 = line_check(x0, y0, x1, y1, x2, y2, False, False)
    chk2 = line_check(x0, y0, x2, y2, x3, y3, False, True)
    chk3 = line_check(x0, y0, x3, y3, x4, y4, True, True)
    chk4 = line_check(x0, y0, x4, y4, x1, y1, True, False)

    # check if point is within resitected half place side of all lines --> in object
    if chk1 and chk2 and chk3 and chk4:
        return False  # point is in obstacle space
    else:
        return True  # point is not in obstacle space


def line_check(x0, y0, x1, y1, x2, y2, UD, LR):
    # UD = True  if object is bottom side of line
    # UD = False if object is top    side of line
    # LR = True  if object is left   side of line
    # LR = False if object is right  side of line
    if x2 != x1:  # not vertical line
        m = (y2 - y1) / (x2 - x1)  # get the slope
        b = y1 - m * x1  # get the intercept
        # check if point is within the restriced half-plane
        if (y0 >= m * x0 + b and not UD) or (y0 <= m * x0 + b and UD):
            return True  # True means point is within the restriced half-plane
        else:
            return False  # False means point is not within the restriced half-plane

    else:  # x2 == x1 --> vertical line
        if (x0 >= x1 and not LR) or (x0 <= x1 and LR):
            return True  # True means point is within the restriced half-plane
        else:
            return False  # False means point is not within the restriced half-plane


def elip_check(x0, y0, elip):
    # extract dimensions out of elip list
    xc = elip[0]
    yc = elip[1]
    a2 = elip[2]**2  # horizontal dimension
    b2 = elip[3]**2  # vertical dimension

    if (x0 - xc)**2 / a2 + (y0 - yc)**2 / b2 <= 1:  # check if point is within ellipse
        return False  # point is in obstacle space
    else:
        return True  # point is not in obstacle space


def setup_graph(clearance, point_robot = True):
    obst = np.ones((GRID_H,GRID_W))
    for x in range(GRID_W):
        for y in range(GRID_H):
            for quad in quads:  # check quads
                if not quad_check(x, y, quad):  # see if point is near the quad
                    obst[GRID_H-y, x] = 0
                    break
    
            for elip in elips:  # check elips
                if not elip_check(x, y, elip):  # see if point is near the elip
                    obst[GRID_H-y, x] = 0
                    break
                
    if not point_robot:  # used to override the expansion for the visualization step
        return obst
                       
    newObst = np.ones((GRID_H,GRID_W))  # create new obstacle array that will have r
    r = int(round(GRID_ROBOT_RADIUS + clearance))  # point robot radius
    for x in range(GRID_W):
        for y in range(GRID_H):
            for i in range(x-r, x+r):  # window each pixel and check for an obstacle in radius
                for j in range(y-r, y+r):
                    if 0 <= i < GRID_W and 0 <= j < GRID_H:  # makes sure point is within bounds
                        if obst[j, i] == 0 and sqrt((x-i)**2+(y-j)**2) < r:  # if window point is in obstacle
                            newObst[y, x] = 0
                            break
                else:
                    continue
                break
            
    return newObst            

# Xi, Yi, Thetai: Input point's coordinates
# Xn, Yn, Thetan: End point coordintes
# UL, UR: Left and rigth wheel velocities
# r: radius of wheels
# L: lateral distance between wheels
def cost(Xi,Yi,Thetai,UL,UR,r,L):
    dt = 0.1
    t = 0
    Xn = Xi
    Yn = Yi
    Thetan = PI * Thetai / 180
    D = 0
    while t < 1:
        dd = 0.5 * r * (UL + UR) * dt
        dXn = dd * cos(Thetan)
        dYn = dd * sin(Thetan)
        Xn += dXn
        Yn += dYn
        Thetan += ((r / L) * (UR - UL) * dt)
        D += sqrt(dXn**2 + dYn**2)
        t += dt
    Thetan = 180 * Thetan / PI
    return Xn, Yn, Thetan, D, t

# A single state of the maze's search
class MazeVertexNode(object):

    # The wheel speeds that can be used to get to neighbors, in (L,R) pairs
    WHEEL_SPEEDS_TO_NEIGHBORS = (
        (0, 1),
        (0, 2),
        (1, 2),
        (1, 1),
        (2, 2),
        (1, 0),
        (2, 0),
        (2, 1)
    )

    # The parent MazeVertexNode to this instance
    parent = None

    # A 3-tuple, (y,x,t) coordinate pair, where t is an orientation theta
    position = None

    # The current tentative distance from the start to this node
    distG = None

    # The current tentative distance from this node to the goal, given some heuristic
    distF = None

    # The wheel speeds and time duration used to visit this node (dt, theta_l, theta_r)
    twist_elements_to_here = None

    # Constructor, given all values
    def __init__(self, parent, position, distG, distF):
        self.parent = parent
        self.position = position
        self.distG = distG
        self.distF = distF
        self.twist_elements_to_here = (0,0,0)

# A link in a priorty queue chain
class DoublyLinkNode(object):

    # An instance of MazeVertexNode
    vertex_node = None

    # The left / lower priority DoublyLinkNode
    link_parent = None

    # The right / higher priority DoublyLinkNode
    link_child = None

    # Create a link with a vertex node and parent
    def __init__(self, vertex_node, link_parent):
        self.vertex_node = vertex_node
        self.link_parent = link_parent
        self.link_child = None
        if self.link_parent != None:
            self.link_parent.link_child = self

    # Helper function to remove this node from a chain it's in
    def remove_from_chain(self):
        if self.link_parent != None:
            self.link_parent.link_child = self.link_child
        if self.link_child != None:
            self.link_child.link_parent = self.link_parent
        self.link_parent = None
        self.link_child = None

# A collection of a couple of physical characteristics of the robot
class RobotDescription(object):

    # The wheel speed multipliers
    wheel_speeds = None

    # The wheel radius
    r = None

    # The lateral distance between the wheels
    L = None

    # Collect data about the robot
    def __init__(self, wheel_speed_minor, wheel_speed_major, r, L):
        self.wheel_speeds = (0, wheel_speed_minor, wheel_speed_major)
        self.r = r
        self.L = L

# A pathfinding object which builds a representation of the underlying maze
class Maze(object):

    # The DiscreteGraph representation of the maze
    obst = None

    # The wheel speed multiplier
    robo_desc = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, clearance, wheel_speed_minor, wheel_speed_major, r, L):
        self.obst = setup_graph(clearance)  # creates the obstacle space. 0 for obstacle, 1 for open space
        self.robo_desc = RobotDescription(wheel_speed_minor, wheel_speed_major, r, L)

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, j, i):
        sh = self.obst.shape
        return (j >= 0) and (i >= 0) and (j < sh[0]) and (i < sh[1]) and (self.obst[j,i] == 1)

    # Calculate the distance between two points
    def dist(self, n1, n2):
        return sqrt((n2[1]-n1[1])**2 + (n2[0]-n1[0])**2)

    # Calculate the tentative remaining distance from n to goal, given this heuristic
    def h(self, n, goal):
        return self.dist(n, goal)

    # Run A* between a start and goal point, using a forward step length
    def astar(self, start, goal):
        link_node_indices = {}
        prev_n = None
        for j in range(GRID_H):
            for i in range(GRID_W):
                if self.is_in_board(j, i):
                    for o in range(GRID_O):
                        v = (j,i,o)
                        if v != start:
                            n = DoublyLinkNode(MazeVertexNode(None, v, 999999999, 999999999), prev_n)
                            link_node_indices[v] = n
                            prev_n = n
        start_node = DoublyLinkNode(MazeVertexNode(None, start, 0, self.h(start, goal)), prev_n)
        link_node_indices[start] = start_node

        # The next node to process in the priority queue, initialized to be the start node
        node_to_visit = start_node

        # Track the nodes that were visited in the order they were, to visualize later
        nodes_visited = []

        # Start the main part of the algorithm, tracking the node that can be used to recover the path
        final_node = None
        idx = 0
        while (final_node is None) and (node_to_visit != None):
            # Essentially, mark this node as "visited" and capture its position
            node = node_to_visit
            np = node.vertex_node.position
            del link_node_indices[np]

            # Check if this is the goal position
            if self.dist(np, goal) <= 2:
                final_node = node.vertex_node
                continue

            # Track the moves used to explore the neighbors
            moves_to_neighbors = []

            # Get each of the neighbors of this node by looping through the five possible actions
            nj, ni, orientation = np
            for Uln,Urn in MazeVertexNode.WHEEL_SPEEDS_TO_NEIGHBORS:
                Ul = self.robo_desc.wheel_speeds[Uln]
                Ur = self.robo_desc.wheel_speeds[Urn]
                ii, jj, ori, nD, dt = cost(ni, nj, orientation*BOARD_O, Ul, Ur, self.robo_desc.r, self.robo_desc.L)
                ii = int(ii)
                jj = int(jj)
                ori = (int(ori) % 360) // BOARD_O
                neighbor = (jj,ii,ori)

                neighbor_node = link_node_indices.get(neighbor, None)
                if neighbor_node is None:
                    # This node was already visited and removed, continue to the next neighbor
                    continue

                # (Re)set the twist elements used to get here
                neighbor_node.vertex_node.twist_elements_to_here = (Ul, Ur, dt)

                # Add the position of this neighbor to visualize later
                moves_to_neighbors.append(MoveCommand(
                    left_wheel_speed=Ul,
                    right_wheel_speed=Ur,
                    time_elapsed=dt
                ))

                # Calculate the adjusted distance
                node_distG = node.vertex_node.distG + nD
                if node_distG < neighbor_node.vertex_node.distG:
                    # Set this node as this neighbor's shortest path
                    neighbor_node.remove_from_chain()
                    neighbor_node.vertex_node.distG = node_distG
                    neighbor_node.vertex_node.distF = node_distG + self.h(neighbor, goal)
                    neighbor_node.vertex_node.parent = node.vertex_node
                    # Do a less costly sort by simply moving the neighbor node in the list
                    # If early on in the program, just search from right to left
                    potential_new_link_parent = node_to_visit
                    while True:
                        if potential_new_link_parent.vertex_node.distF >= neighbor_node.vertex_node.distF:
                            # Found where the node should be placed in the chain, insert it
                            neighbor_node.link_parent = potential_new_link_parent
                            neighbor_node.link_child = potential_new_link_parent.link_child
                            if potential_new_link_parent.link_child != None:
                                potential_new_link_parent.link_child.link_parent = neighbor_node
                            potential_new_link_parent.link_child = neighbor_node
                            break
                        elif potential_new_link_parent.link_parent is None:
                            # This node becomes the new end (lowest priority)
                            neighbor_node.link_parent = None
                            neighbor_node.link_child = potential_new_link_parent
                            potential_new_link_parent.link_parent = neighbor_node
                            break
                        else:
                            # Have not found the end yet
                            potential_new_link_parent = potential_new_link_parent.link_parent

            # Add this position as having visited each of these neighbors
            nodes_visited.append(NeighborsPose2D(
                position=Pose2D(x=ni, y=nj, theta=orientation*BOARD_O),
                moves_to_neighbors=moves_to_neighbors
            ))
            idx = idx + 1
            printr("Planning{0}".format("." * (idx // 2000)))

            # Continue to the next node
            node_to_visit = node_to_visit.link_parent
            node_to_visit.link_child = None

        print
        return final_node, nodes_visited

def handle_plan_request(msg, maze, pub):
    path_msg = Path()
    path_msg.success = False
    path_msg.request = msg

    valid_inputs = True

    s = None
    try:
        s = int(msg.init_position.y*GRID_D), int(msg.init_position.x*GRID_D), 0
    except:
        print "Please enter the start position in \"y,x\" format, where y and x are numbers."
        valid_inputs = False

    g = None
    try:
        g = int(msg.final_position.y*GRID_D), int(msg.final_position.x*GRID_D), 0
    except:
        print "Please enter the goal position in \"y,x\" format, where y and x are numbers."
        valid_inputs = False

    if valid_inputs and maze.is_in_board(s[0], s[1]) and maze.is_in_board(g[0], g[1]):
        print "Running A* with {0} => {1}...".format(s, g)
        final_node, nodes_visited = maze.astar(s,g)
        print "Finished, visited {0}".format(len(nodes_visited))

        backtrack_path = []
        n = final_node
        while n is not None:
            backtrack_path.append(BacktrackNode(
                position=Pose2D(x=n.position[1], y=n.position[0], theta=n.position[2]*BOARD_O),
                has_move_cmd=(n.parent is not None),
                move_cmd=MoveCommand(
                    left_wheel_speed=n.twist_elements_to_here[0],
                    right_wheel_speed=n.twist_elements_to_here[1],
                    time_elapsed=n.twist_elements_to_here[2]
                )
            ))
            n = n.parent

        path_msg.success = (final_node is not None)
        path_msg.explored = nodes_visited
        path_msg.backtrack_path = backtrack_path
#        visualize(final_node, nodes_visited)
    else:
        print "Failed to run A* with {0} => {1}".format(s, g)

    pub.publish(path_msg)

def cleanly_handle_plan_request(msg, maze, pub):
    handle_plan_request(msg, maze, pub)
    gc_collect()

def main():
    # Capture required user input
    my_sargv = rospy.myargv(argv=sargv)
    assert(len(my_sargv) == 4)
    c_str, w1_str, w2_str = my_sargv[1:]

    clearance = None
    try:
        clearance = int(c_str)
    except:
        print "Please enter the clearance as an integer."
        return
    if clearance < 0:
        print "Please enter the clearance as a non-negative integer."
        return

    wheel_speed_minor = None
    wheel_speed_major = None
    try:
        wheel_speed_minor = int(w1_str)
        wheel_speed_major = int(w2_str)
    except:
        print "Please enter the wheel velocities as an integer."
        return
    if (wheel_speed_minor <= 0) or (wheel_speed_major <= 0):
        print "Please enter the robot radius as a positive integer."
        return

    # Init ROS elements and get parameters
    rospy.init_node(ROS_NODE_NAME)
    robot_r = get_rosparam("/nhr/robot_description/r")
    robot_L = get_rosparam("/nhr/robot_description/L")
    assert((robot_r != None) and (robot_L != None))

    # Build the maze and underlying graph object
    print "Starting maze generation..."
    maze = Maze(clearance, wheel_speed_minor, wheel_speed_major, robot_r, robot_L)
    print "Done, waiting for path planning requests."

    # Init ROS pub and sub
    path_pub = rospy.Publisher(
        "/nhr/path",
        Path,
        queue_size=1
    )
    plan_request_sub = rospy.Subscriber(
        "/nhr/plan_request",
        PlanRequest,
        lambda m: cleanly_handle_plan_request(m, maze, path_pub),
        queue_size=1
    )

    rospy.spin()

if __name__ == "__main__":
    main()
