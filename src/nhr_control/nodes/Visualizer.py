#!/usr/bin/env python

from __future__ import print_function
import rospy
from nhr_msgs.msg import Path

ROS_NODE_NAME = "nhr_visualizer"

def handle_path(msg):
    ##############################################################
    # TEMPORARY: print the nodes visited
    # @Alec, feel free to delete me once you've gotten the new viz
    for i,p2d in enumerate(msg.backtrack_path):
        if i != 0:
            print(" => ", end="")
        print("{0},{1},{2}".format(p2d.y, p2d.x, p2d.theta), end="")
    print()
    ##############################################################

#    vid_name = "temp"
#
#    scl = 8  # output video scale factor
#    video_size = (GRID_W*scl, GRID_H*scl)
#    # Build video writer to render the frames at 120 FPS
#    vid_write = cv2.VideoWriter(
#        "{0}.mp4".format(vid_name),
#        cv2.VideoWriter_fourcc(*'mp4v'),
#        30.0,
#        video_size
#    )
#    # Build image to be white and draw the obstacles
#    temp = np.uint8(setup_graph(clearance, point_robot=False))
#    temp *= 255
#    img = np.empty((GRID_H,GRID_W,3), dtype=np.uint8)
#    img[:, :, 0] = temp
#    img[:, :, 1] = temp
#    img[:, :, 2] = temp
#    img[s[:2]] = (0,255,0)
#    img[g[0]-2:g[0]+2,g[1]-2:g[1]+2] = (0,0,255)
#    img = cv2.resize(img, video_size, interpolation=cv2.INTER_NEAREST)
#
#    # Go through the pixels visited
#    N = len(nodes_visited)
#    for i in range(N):
#        conn = nodes_visited[i]
#        src = conn[0]
#        for dest in conn[1]:
#            img = cv2.line(
#                img,
#                (int(src[0]/2*scl), int(src[1]/2*scl)),
#                (int(dest[0]/2*scl), int(dest[1]/2*scl)),
#                (255,120,120),
#                1
#            )
#
#        # ramp up video search speed
#        if i<900 and i<N/100 and i%10==0:
#            vid_write.write(img)
#        elif i<N/10 and i%300==0:
#            vid_write.write(img)
#        elif i%1000 or i==N-1:
#            vid_write.write(img)
#
#    # Draw the final path
#    img = cv2.line(
#        img,
#        (int(path_node.position[1]/2*scl), int(path_node.position[0]/2*scl)),
#        (int(path_node.parent.position[1]/2*scl), int(path_node.parent.position[0]/2*scl)),
#        (0,0,255),
#        1
#    )
#    for i in range(10):
#        vid_write.write(img)
#    path_node = path_node.parent
#    while path_node.parent is not None:
#        img = cv2.line(
#            img,
#            (int(path_node.position[1]/2*scl), int(path_node.position[0]/2*scl)),
#            (int(path_node.parent.position[1]/2*scl), int(path_node.parent.position[0]/2*scl)),
#            (0,0,255),
#            1
#        )
#        for i in range(10):
#            vid_write.write(img)
#        path_node = path_node.parent
#    vid_write.release()
#    print "Finished."

def main():
    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    path_sub = rospy.Subscriber(
        "/nhr/path",
        Path,
        handle_path,
        queue_size=1
    )
    print("Ready, waiting for paths...")
    rospy.spin()

if __name__ == "__main__":
    main()
