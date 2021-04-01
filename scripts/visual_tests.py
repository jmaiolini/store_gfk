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

    filename = os.path.splitext(filepath)[0]+'.jpg'
    robot_stops_img = cv2.imread(filename,1)

    for pose in robot_trajectory:
        x,y = utils.map2image(pose[0],pose[1])
        robot_stops_img = cv2.circle(robot_stops_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
    
    utils.show_img_and_wait_key("Trajectories",robot_stops_img)

    check_traj_blender_map(robot_trajectory,x_ratio,y_ratio)

def check_traj_blender_map(robot_trajectory,x_ratio, y_ratio):
    utils = Utils()
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm' 
    robot_stops_img = cv2.imread(filepath,1)

    for pose in robot_trajectory:
        x,y = utils.map2image(pose[0],pose[1]) #TODO!!
        x += 5
        y -= 5
        robot_stops_img = cv2.circle(robot_stops_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

    robot_stops_img = cv2.circle(robot_stops_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_stops_img = cv2.circle(robot_stops_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)

    utils.show_img_and_wait_key("Trajectories",robot_stops_img)

    
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
    # img = np.zeros([512,512,1],dtype=np.uint8)
    # img_rgb = np.zeros([512,512,3],dtype=np.uint8)
    # img.fill(255)
    # img_rgb.fill(255)
    offset = 200
    #occupied roi
    # img[50:90,100:200].fill(0)
    # img[100:200,50:90].fill(0)
    # img[50+offset:90+offset,100+offset:200+offset].fill(0)
    # img[100+offset:200+offset,50+offset:90+offset].fill(0)

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
