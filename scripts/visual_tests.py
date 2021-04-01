#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2
import numpy as np
from utils import Utils

#TODO: parametrize map path (need to add arg)

###### VISUAL TESTS (trajectories and recovery)
def check_traj_correspondences( robot_trajectory, filename, x_ratio, y_ratio):

    utils = Utils()
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/' + filename
    blender_filename = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'

    filename = os.path.splitext(filepath)[0]+'.jpg'
    robot_stops_img = cv2.imread(filename,1)
    robot_stops_blender_img = cv2.imread(blender_filename,1)

    for pose in robot_trajectory:

        pose_x_on_blender = utils.maps_x_ratio * pose[0]
        pose_y_on_blender = utils.maps_y_ratio * pose[1]

        x,y = utils.map2image(pose[0],pose[1])
        x_blender,y_blender = utils.map2image_blender(pose_x_on_blender,pose_y_on_blender)   
        robot_stops_img = cv2.circle(robot_stops_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)
        robot_stops_blender_img = cv2.circle(robot_stops_blender_img, (x_blender,y_blender), radius=5, color=(0, 0, 255), thickness=-1)


    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
    
    utils.show_img_and_wait_key("Trajectories",robot_stops_img)
    utils.show_img_and_wait_key("Trajectories2",robot_stops_blender_img)


    
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

    utils.show_img_and_wait_key("Goals Tollerance",robot_stops_img)

def check_feasibility():
    utils = Utils()

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'

    img_rgb = cv2.imread(filepath,cv2.IMREAD_COLOR)
    img = cv2.imread(filepath,cv2.IMREAD_GRAYSCALE)

    #desired waypoints
    inside_points = [(150,355), (600,735), (680,200),(500,500)]
    # inside_points2 = [(53,110),(193,70),(78+offset,183+offset),(150+offset,70+offset)]
    for point in inside_points:
        cv2.circle(img_rgb, point, radius=3, color=(0, 0, 255), thickness=2)
        new_point = utils.find_closest_goal(img, point)
        cv2.circle(img_rgb, new_point, radius=3, color=(0, 255, 0), thickness=2)

    # for point in inside_points2:
    #     cv2.circle(img_rgb, point, radius=3, color=(0, 255, 255), thickness=2)
    #     new_point = utils.find_closest_goal(img, point)
    #     cv2.circle(img_rgb, new_point, radius=3, color=(255, 255, 0), thickness=2)

    utils.show_img_and_wait_key("Feasibility", img_rgb)
