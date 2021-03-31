#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2
import numpy as np
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
    filepath = rospack.get_path('store_gfk') + '/trajectories/' + filename

    filename = os.path.splitext(filepath)[0]+'.jpg'
    robot_stops_img = cv2.imread(filename,1)

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
    filepath = rospack.get_path('store_gfk') + '/trajectories/' + filename
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

def check_feasibility():
    utils = Utils()
    img = np.zeros([512,512,1],dtype=np.uint8)
    img_rgb = np.zeros([512,512,3],dtype=np.uint8)
    img.fill(255)
    img_rgb.fill(255)
    offset = 200
    #occupied roi
    img[50:90,100:200].fill(0)
    img[100:200,50:90].fill(0)
    img[50+offset:90+offset,100+offset:200+offset].fill(0)
    img[100+offset:200+offset,50+offset:90+offset].fill(0)
    img_rgb[50:90,100:200].fill(0)
    img_rgb[100:200,50:90].fill(0)
    img_rgb[50+offset:90+offset,100+offset:200+offset].fill(0)
    img_rgb[100+offset:200+offset,50+offset:90+offset].fill(0)

    #shift in meters
    shift = 0.30 
    #desired waypoints
    inside_points = [(70,150),(150,70),(70+offset,150+offset),(150+offset,70+offset)]
    inside_points2 = [(53,110),(193,70),(78+offset,183+offset),(150+offset,70+offset)]
    for point in inside_points:
        cv2.circle(img_rgb, point, radius=2, color=(0, 0, 255), thickness=2)
        new_point = utils.find_closest_goal(img, point, False, shift)
        cv2.circle(img_rgb, new_point, radius=2, color=(0, 255, 0), thickness=2)

    for point in inside_points2:
        cv2.circle(img_rgb, point, radius=2, color=(0, 255, 255), thickness=2)
        new_point = utils.find_closest_goal(img, point, True, shift)
        cv2.circle(img_rgb, new_point, radius=2, color=(255, 255, 0), thickness=2)


    cv2.namedWindow("Feasibility",cv2.WINDOW_NORMAL)
    cv2.imshow('Feasibility', img_rgb)
    cv2.waitKey(0)
    cv2.namedWindow("image",cv2.WINDOW_NORMAL)
    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#TODO: parametrize map path (need to add arg)
# def check_map():
#     rospack = rospkg.RosPack()
#     filepath = rospack.get_path('store_gfk') + '/maps/map.pgm' 
#     map_ = cv2.imread(filepath,1)
#     cv2.circle(map_, (map_.shape[0]/2,map_.shape[1]/2), radius=2, color=(255, 0, 0), thickness=2)
#     cv2.namedWindow("Map",cv2.WINDOW_NORMAL)
#     cv2.resizeWindow("Map", map_.shape[0], map_.shape[1])
#     cv2.imshow("Map", map_)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()