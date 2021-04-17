#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from math import sqrt, cos, sin, pi as PI
from Cost import cost

ROS_NODE_NAME = "nhr_planner"

BOARD_H = 10
BOARD_W = 10
BOARD_O = 30

GRID_H = BOARD_H * 10
GRID_W = BOARD_W * 10
GRID_O = int(360 / BOARD_O)

DEG2RAD = PI / 180

# Board Obstacles
quads = [[36.53, 124.38, 48, 108, 170.87, 194.04, 159.40, 210.42],
         [200,280,200,230,210,230,210,280],
         [210,280,210,270,230,270,230,280],
         [210,240,210,230,230,230,230,240]]
elips = [[90, 70, 35, 35],
         [246, 145, 60, 30]]


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
    

def setup_graph(robot_radius, clearance, point_robot = True):
    obst = np.ones((BOARD_H,BOARD_W))
    for x in range(BOARD_W):
        for y in range(BOARD_H):
            for quad in quads:  # check quads
                if not quad_check(x, y, quad):  # see if point is near the quad
                    obst[BOARD_H-y, x] = 0
                    break
    
            for elip in elips:  # check elips
                if not elip_check(x, y, elip):  # see if point is near the elip
                    obst[BOARD_H-y, x] = 0
                    break
                
    if not point_robot:  # used to override the expansion for the vizualization step
        return obst
                       
    newObst = np.ones((BOARD_H,BOARD_W))  # create new obstacle array that will have r
    r = robot_radius + clearance  # point robot radius
    for x in range(BOARD_W):
        for y in range(BOARD_H):
            for i in range(x-r, x+r):  # window each pixel and check for an obstacle in radius
                for j in range(y-r, y+r):
                    if i >= 0 and j >= 0 and i < BOARD_W and j < BOARD_H:  # makes sure point is within bounds
                        if obst[j, i] == 0 and sqrt((x-i)**2+(y-j)**2) < r:  # if window point is in obstacle
                            newObst[y, x] = 0
                            break
                else:
                    continue
                break
            
    return newObst            


# A single state of the maze's search
class MazeVertexNode(object):

    # The multiplier for the wheel speeds used in WHEEL_SPEEDS_TO_NEIGHBORS
    WHEEL_SPEED_MAGNITUDE = 50

    # The wheel speeds that can be used to get to neighbors, in (L,R) pairs
    WHEEL_SPEEDS_TO_NEIGHBORS = (
        (0,                       WHEEL_SPEED_MAGNITUDE  ),
        (0,                       2*WHEEL_SPEED_MAGNITUDE),
        (WHEEL_SPEED_MAGNITUDE,   2*WHEEL_SPEED_MAGNITUDE),
        (WHEEL_SPEED_MAGNITUDE,   WHEEL_SPEED_MAGNITUDE  ),
        (2*WHEEL_SPEED_MAGNITUDE, 2*WHEEL_SPEED_MAGNITUDE),
        (WHEEL_SPEED_MAGNITUDE,   0                      ),
        (2*WHEEL_SPEED_MAGNITUDE, 0                      ),
        (2*WHEEL_SPEED_MAGNITUDE, WHEEL_SPEED_MAGNITUDE  )
    )

    # The parent MazeVertexNode to this instance
    parent = None

    # A 3-tuple, (y,x,t) coordinate pair, where t is an orientation theta
    position = None

    # The current tentative distance from the start to this node
    distG = None

    # The current tentative distance from this node to the goal, given some heuristic
    distF = None

    # Constructor, given all values
    def __init__(self, parent, position, distG, distF):
        self.parent = parent
        self.position = position
        self.distG = distG
        self.distF = distF

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

# A pathfinding object which builds a representation of the underlying maze
class Maze(object):

    # The DiscreteGraph representation of the maze
    obst = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, robot_radius, clearance):
        self.obst = setup_graph(robot_radius, clearance)  # creates the obstacle space. 0 for obstacle, 1 for open space

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
    def astar(self, start, goal, step):
        start = start[0]*2, start[1]*2, start[2]
        goal = goal[0]*2, goal[1]*2, goal[2]
        link_node_indices = {}
        prev_n = None
        for j in range(GRID_H):
            for i in range(GRID_W):
                if self.is_in_board(int(j/2), int(i/2)):
                    for o in range(GRID_O):
                        v = (j,i,o)
                        if v != start:
                            n = DoublyLinkNode(MazeVertexNode(None, v, 999999999, 999999999), prev_n)
                            link_node_indices[v] = n
                            prev_n = n
            print "\r  - building initial priority queue for A*: {0}/{1}".format(j+1, GRID_H)
        print
        start_node = DoublyLinkNode(MazeVertexNode(None, start, 0, self.h(start, goal)), prev_n)
        link_node_indices[start] = start_node

        # The next node to process in the priority queue, initialized to be the start node
        node_to_visit = start_node

        # Track the nodes that were visited in the order they were, to visualize later
        nodes_visited = []

        # Start the main part of the algorithm, tracking the node that can be used to recover the path
        final_node = None
        pxidx = 0
        while (final_node is None) and (node_to_visit != None):
            # Essentially, mark this node as "visited" and capture its position
            node = node_to_visit
            np = node.vertex_node.position
            del link_node_indices[np]

            # Check if this is the goal position
            if self.dist(np, goal) <= 1.5*step:
                final_node = node.vertex_node
                continue

            # Track the neighbors of this node that were explored
            neighbors_explored = []

            # Get each of the neighbors of this node by looping through the five possible actions
            nj, ni, orientation = np
            for Ul,Ur in MazeVertexNode.WHEEL_SPEEDS_TO_NEIGHBORS:
                ii, jj, ori, nD = cost(ni, nj, orientation, Ul, Ur)
                ii = int(ii)
                jj = int(jj)
                ori = ori // BOARD_O
                neighbor = (jj,ii,ori)

                neighbor_node = link_node_indices.get(neighbor, None)
                if neighbor_node is None:
                    # This node was already visited and removed, continue to the next neighbor
                    continue

                # Add the position of this neighbor to visualize later
                neighbors_explored.append((ii, jj))

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
            nodes_visited.append(((ni, nj), neighbors_explored))
            pxidx = pxidx + 1
            print "\rVisited {0}".format(pxidx)

            # Continue to the next node
            node_to_visit = node_to_visit.link_parent
            node_to_visit.link_child = None

        # If there's no path, the final_node will be null, but nodes_visited could still have content
        return final_node, nodes_visited

def main():
    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)

    # Capture required user input
    s = None
    try:
        s_str = raw_input("Enter the start position: ")
        s_comma = s_str.index(",")
        s = int(s_str[:s_comma]), int(s_str[s_comma+1:]), 0
    except:
        print "Please enter the start position in \"y,x\" format, where y and x are integers."
        return

    g = None
    try:
        g_str = raw_input("Enter the goal position: ")
        g_comma = g_str.index(",")
        g = int(g_str[:g_comma]), int(g_str[g_comma+1:]), 0
    except:
        print "Please enter the goal position in \"y,x\" format, where y and x are integers."
        return

    robot_radius = None
    try:
        r_str = raw_input("Enter the robot radius: ")
        robot_radius = int(r_str)
    except:
        print "Please enter the robot radius as an integer."
        return
    if robot_radius <= 0:
        print "Please enter the robot radius as a positive integer."
        return

    clearance = None
    try:
        c_str = raw_input("Enter the clearance: ")
        clearance = int(c_str)
    except:
        print "Please enter the clearance as an integer."
        return
    if clearance < 0:
        print "Please enter the clearance as a non-negative integer."
        return

    step = None
    try:
        t_str = raw_input("Enter the robot movement step (integer between 1 and 10, inclusive): ")
        step = int(t_str)
    except:
        print "Please enter an integer."
        return
    if (step < 1) or (step > 10):
        print "Please enter an integer between 1 and 10, inclusive."
        return

    vid_name = raw_input("Enter the name of the output file (no file extension, ex. 'output1'): ")

    # Build the maze and underlying graph object
    print "Starting maze generation..."
    maze = Maze(robot_radius, clearance)
    # Check if they're traversable positions in the maze, continue if so
    if maze.is_in_board(s[0],s[1]) and maze.is_in_board(g[0],g[1]):
        # Do Dijkstra
        print "Done. Planning path..."
        path_node, nodes_visited = maze.astar(s,g,step)
        print "Done (visited {0} positions). Starting render...".format(len(nodes_visited))
        
        scl = 4  # output video scale factor
        # Build video writer to render the frames at 120 FPS
        vid_write = cv2.VideoWriter(
            "{0}.mp4".format(vid_name),
            cv2.VideoWriter_fourcc(*'mp4v'),
            30.0,
            (BOARD_W*scl, BOARD_H*scl)
        )
        # Build image to be white and draw the obstacles
        temp = np.uint8(setup_graph(robot_radius, clearance, point_robot = False))
        temp *= 255
        img = np.empty((BOARD_H,BOARD_W,3), dtype=np.uint8)
        img[:, :, 0] = temp
        img[:, :, 1] = temp
        img[:, :, 2] = temp
        img[s[:2]] = (0,255,0)
        img[g[0]-2:g[0]+2,g[1]-2:g[1]+2] = (0,0,255)
        img = cv2.resize(img, (BOARD_W*scl, BOARD_H*scl), interpolation = cv2.INTER_NEAREST)

        # Go through the pixels visited
        N = len(nodes_visited)
        for i in range(N):
            conn = nodes_visited[i]
            src = conn[0]
            for dest in conn[1]:
                img = cv2.line(
                    img,
                    (int(src[0]/2*scl), int(src[1]/2*scl)),
                    (int(dest[0]/2*scl), int(dest[1]/2*scl)),
                    (255,120,120),
                    1
                )
            
            # ramp up video search speed
            if i<900 and i<N/100 and i%10==0:
                vid_write.write(img)
            elif i<N/10 and i%300==0:
                vid_write.write(img)
            elif i%1000 or i==N-1:
                vid_write.write(img)
            
        # Draw the final path
        img = cv2.line(
            img,
            (int(path_node.position[1]/2*scl), int(path_node.position[0]/2*scl)),
            (int(path_node.parent.position[1]/2*scl), int(path_node.parent.position[0]/2*scl)),
            (0,0,255),
            1
        )
        for i in range(10):
            vid_write.write(img)
        path_node = path_node.parent
        while path_node.parent is not None:
            img = cv2.line(
                img,
                (int(path_node.position[1]/2*scl), int(path_node.position[0]/2*scl)),
                (int(path_node.parent.position[1]/2*scl), int(path_node.parent.position[0]/2*scl)),
                (0,0,255),
                1
            )
            for i in range(int(step/2)):
                vid_write.write(img)
            path_node = path_node.parent
        vid_write.release()
        print "Finished."
    else:
        print "Either the start {0} or the goal {1} was not valid.".format(s, g)

if __name__ == "__main__":
    main()
