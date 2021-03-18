#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2

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


#TODO modify them based on the store (for now only store B available)
# px_m_ratio_x = 0.026529338 #m/px WRONG
# px_m_ratio_y = 0.02677686 #m/px WRONG


store_px_w = 1602 #store pixels width 
store_px_h = 1210 #store pixels height
store_m_w = 42.5 #store meter width
store_m_h = 32.4 #store meter height



def print_usage(exit_code = 0):
    print '''Usage: rosrun store_gfk wpoints_generator.py <filename>
    
    - <filename> of store you are inspecting must be a 
    json file under trajectories/<store_name>/ directory'''

    sys.exit(exit_code)

def load_trajectory(filename):
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename

 
    if os.path.exists(filepath):
        with open(filepath) as f:
            data = json.load(f)
    else:
        sys.exit('Could not find the trajectory file.\nExiting.')

        
    return transform_waypoints(data["points"])

def transform_waypoints(store_waypoints):
    robot_waypoints = list()
    
    
    planimetry_width = 42.5
    planimetry_height = 32.4
    img_width = 1602
    img_height = 1210

    #TODO need also pixels and map meter sizes to calculate ratio. For now hardcoded

    px_m_ratio_x = planimetry_width / (img_width-22-30)
    px_m_ratio_y = planimetry_height / (img_height-24-26)
 
    store_width_m = img_width * px_m_ratio_x #42.5 
    store_height_m = img_height * px_m_ratio_y #32.4 
    
    
    for coord in store_waypoints: #dict type
        new_coord = tranform_position( coord['x'], coord['y'], store_width_m, store_height_m )
        robot_waypoints.append( new_coord )

    print(robot_waypoints)
    return robot_waypoints, px_m_ratio_x, px_m_ratio_y
    
def tranform_position(x_traj, y_traj, store_width, store_height): 
    # x_traj += x_off
    # y_traj += y_off

    x_map = x_traj 
    y_map = store_height - y_traj
        
    if x_map >= store_width or y_map >= store_height:
        print('Trajectory out of store map. Aborting')
        sys.exit(1)

    if x_map <= 0 or y_map <= 0:
        print('Negative position. Aborting')
        sys.exit(1)

    return ( x_map, y_map )


def pixels2meters(x_px,y_px, x_ratio, y_ratio):
    x_m = x_px * x_ratio
    y_m = y_px * y_ratio

    return x_m,y_m

def meters2pixels(x_m,y_m, x_ratio, y_ratio):
    x_px = x_m / x_ratio
    y_px = y_m / y_ratio

    return x_px,y_px

#changes coordinates of a point in meter from image to map coordinates
def image2map(p_x,p_y, x_ratio, y_ratio):
    x_map = p_x * x_ratio
    y_map = store_m_h - p_y * y_ratio

    return x_m,y_m
#changes coordinates of a point in pixels from map to image coordinates
def map2image(p_x,p_y, x_ratio, y_ratio):
    x_img = int(p_x / x_ratio)
    y_img = store_px_h - int(p_y / y_ratio)

    return x_img,y_img


######TESTS (trajectories diferrence must be around (0.6, 0.65) meter )
def check_traj_correspondences( robot_trajectory, filename, x_ratio, y_ratio):

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename
    filepath = os.path.splitext(filepath)[0]+'.jpg'
    
    robot_traj_img = cv2.imread(filepath,1)

    for pose in robot_trajectory:
        x,y = map2image(pose[0],pose[1], x_ratio, y_ratio)
        robot_traj_img = cv2.circle(robot_traj_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

    robot_traj_img = cv2.circle(robot_traj_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
    robot_traj_img = cv2.circle(robot_traj_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
    cv2.namedWindow("Trajectories",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Trajectories", int(1602/2), int(1210/2))
    cv2.imshow("Trajectories", robot_traj_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/prova.jpg",robot_traj_img)
    


