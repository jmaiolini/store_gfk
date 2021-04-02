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
def check_traj_correspondences( robot_trajectory, filename, map_source):

    utils = Utils(filename, map_source)
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/' + filename
    blender_filename = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'

    filename = os.path.splitext(filepath)[0]+'.jpg'
    robot_stops_img = cv2.imread(filename,1)
    robot_stops_blender_img = cv2.imread(blender_filename,1)

    for pose in robot_trajectory:

        x = utils.maps_x_ratio * pose[0]
        y = utils.maps_y_ratio * pose[1]

        if utils.map_source == 0:
            i,j = utils.map2image(pose[0],pose[1])   
            robot_stops_img = cv2.circle(robot_stops_img, (i,j), radius=5, color=(0, 0, 255), thickness=-1)
        elif utils.map_source == 1:
            i,j = utils.map2image(x,y)
            robot_stops_blender_img = cv2.circle(robot_stops_blender_img, (i,j), radius=3, color=(0, 0, 255), thickness=-1)


    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=3, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=3, color=(0, 0, 255), thickness=-1)

    if utils.map_source == 0:
        utils.show_img_and_wait_key("Original Trajectories",robot_stops_img)
    elif utils.map_source == 1:
        utils.show_img_and_wait_key("Trajectories on blender map",robot_stops_blender_img)


def check_feasibility():
    utils = Utils()

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'

    img_rgb = cv2.imread(filepath,cv2.IMREAD_COLOR)
    img = cv2.imread(filepath,cv2.IMREAD_GRAYSCALE)

    #desired waypoints
    inside_points = [(150,355), (600,735), (680,200),(500,500)]

    for point in inside_points:
        cv2.circle(img_rgb, point, radius=3, color=(0, 0, 255), thickness=2)
        new_point = utils.find_closest_goal(img, point)
        cv2.circle(img_rgb, new_point, radius=3, color=(0, 255, 0), thickness=2)

    utils.show_img_and_wait_key("Feasibility", img_rgb)


def show_good_bad_points(waypoints,map_source):
    utils = Utils()
    averages = []
    ksize = 4 #for each side
    rospack = rospkg.RosPack()
    filename = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'
    img = cv2.imread(filename,cv2.IMREAD_COLOR)

    for waypoint in waypoints:
        x = utils.maps_x_ratio * waypoint[0]
        y = utils.maps_y_ratio * waypoint[1]

        i,j = utils.map2image(x,y)
       
        patch = img[j-ksize:j+ksize,i-ksize:i+ksize,1]
        img[j-ksize:j+ksize,i-ksize:i+ksize] = (0,0,255)
        
        averages.append(np.average(patch))

    utils.show_img_and_wait_key("Wpoints goodness", img)
