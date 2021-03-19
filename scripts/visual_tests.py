#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2
from utils import Utils

#PROBLEM: we have mismatching coordinates as follows
# reasoning to make it generic to images and stores.

#   _____________________________
#   |  ________________________  |
#   | |                        | |
#   | |                        | |
#   | |                        | |
#   | |________________________| |
#   |____________________________|

#we have that:
# the outer is given by image pixels
# the inner is given by the store map




###### VISUAL TESTS (trajectories and recovery)
def check_traj_correspondences( robot_trajectory, filename, x_ratio, y_ratio):

    utils = Utils()
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename
    filepath = os.path.splitext(filepath)[0]+'.jpg'
    
    robot_traj_img = cv2.imread(filepath,1)

    for pose in robot_trajectory:
        x,y = utils.map2image(pose[0],pose[1])
        robot_traj_img = cv2.circle(robot_traj_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

    robot_traj_img = cv2.circle(robot_traj_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_traj_img = cv2.circle(robot_traj_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
    cv2.namedWindow("Trajectories",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Trajectories", int(1602/2), int(1210/2))
    cv2.imshow("Trajectories", robot_traj_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/prova.jpg",robot_traj_img)
    


