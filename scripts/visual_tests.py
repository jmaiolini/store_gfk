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
    
    robot_stops_img = cv2.imread(filepath,1)

    for pose in robot_trajectory:
        x,y = utils.map2image(pose[0],pose[1])
        robot_stops_img = cv2.circle(robot_stops_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
    cv2.namedWindow("Trajectories",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Trajectories", int(1602/2), int(1210/2))
    cv2.imshow("Trajectories", robot_stops_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/prova.jpg",robot_stops_img)
    
def check_radious(robot_trajectory, filename,radius):

    utils = Utils()
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename
    filepath = os.path.splitext(filepath)[0]+'.jpg'
    
    robot_stops_img = cv2.imread(filepath,1)

    for pose in robot_trajectory:
        x,y = utils.map2image(pose[0],pose[1])
        r = utils.meters2pixels(radius,0)[0]
        robot_stops_img = cv2.circle(robot_stops_img, (x,y), radius=r, color=(255, 0, 0), thickness=2)

    cv2.namedWindow("Goals Tollerance",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Goals Tollerance", int(1602/2), int(1210/2))
    cv2.imshow("Goals Tollerance", robot_stops_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def check_map():
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/map/map.pgm'
    map_ = cv2.imread(filepath,1)
    cv2.circle(map_, (map_.shape[0]/2,map_.shape[1]/2), radius=2, color=(255, 0, 0), thickness=2)
    cv2.namedWindow("Map",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Map", map_.shape[0], map_.shape[1])
    cv2.imshow("Map", map_)
    cv2.waitKey(0)
    cv2.destroyAllWindows()