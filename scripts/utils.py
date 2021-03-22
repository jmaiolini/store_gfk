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


class Utils:

    def __init__(self): #to put in config eventually

        self.planimetry_width = 42.5
        self.planimetry_height = 32.4
        self.img_width = 1602
        self.img_height = 1210

        #with this parameter we select how far the robot can stop if the goal is inside a wall
        self.goal_tollerance = 1 #radius in meters

        #TODO need also pixels and map meter sizes to calculate ratio. For now hardcoded

        self.px_m_ratio_x = self.planimetry_width / (self.img_width-22-30)
        self.px_m_ratio_y = self.planimetry_height / (self.img_height-24-26)
    
        self.store_width = self.img_width * self.px_m_ratio_x #42.5 
        self.store_height = self.img_height * self.px_m_ratio_y #32.4 

    @staticmethod
    def print_usage(exit_code=0):
        print '''Usage: rosrun store_gfk wpoints_generator.py <filename>
        
        - <filename> of store you are inspecting must be a 
        json file under trajectories/<store_name>/ directory'''

        sys.exit(exit_code)

    def load_trajectory(self, filename):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename

    
        if os.path.exists(filepath):
            with open(filepath) as f:
                data = json.load(f)
        else:
            sys.exit('Could not find the trajectory file.\nExiting.')

            
        return self.transform_waypoints(data["points"])

    def transform_waypoints(self, store_waypoints):
        robot_waypoints = list()
        
        
        for coord in store_waypoints: #dict type
            new_coord = self.tranform_position( coord['x'], coord['y'])
            robot_waypoints.append( new_coord )

        print(robot_waypoints)
        return robot_waypoints
        
    def tranform_position(self, x_traj, y_traj): 
        # x_traj += x_off
        # y_traj += y_off

        x_map = x_traj 
        y_map = self.store_height - y_traj
            
        if x_map >= self.store_width or y_map >= self.store_height:
            print('Trajectory out of store map. Aborting')
            sys.exit(1)

        if x_map <= 0 or y_map <= 0:
            print('Negative position. Aborting')
            sys.exit(1)

        return ( x_map, y_map )


    def pixels2meters(self,x_px,y_px):
        x_m = x_px * self.px_m_ratio_x
        y_m = y_px * self.px_m_ratio_y

        return x_m,y_m

    def meters2pixels(self,x_m,y_m):
        x_px = int(x_m / self.px_m_ratio_x)
        y_px = int(y_m / self.px_m_ratio_y)

        return x_px,y_px

    #changes coordinates of a point in meter from image to map coordinates
    def image2map(self,p_x,p_y):
        x_map = p_x * self.px_m_ratio_x
        y_map = self.store_height - p_y * self.px_m_ratio_y

        return x_map,y_map
    #changes coordinates of a point in pixels from map to image coordinates
    def map2image(self,p_x,p_y):
        x_img = int(p_x / self.px_m_ratio_x)
        y_img = self.img_height - int(p_y / self.px_m_ratio_y)

        return x_img,y_img
    
    def get_ratios(self):
        return self.px_m_ratio_x, self.px_m_ratio_y
    
    def get_goal_tollerance_r(self):
        return self.goal_tollerance

    ###### VISUAL TESTS (trajectories and recovery)
    def check_traj_correspondences(self,robot_trajectory, filename, x_ratio, y_ratio):

        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk') + '/trajectories/edeka/' + filename
        filepath = os.path.splitext(filepath)[0]+'.jpg'
        
        robot_traj_img = cv2.imread(filepath,1)

        for pose in robot_trajectory:
            x,y = map2image(pose[0],pose[1])
            robot_traj_img = cv2.circle(robot_traj_img, (x,y), radius=5, color=(0, 0, 255), thickness=-1)

        robot_traj_img = cv2.circle(robot_traj_img, (22,24), radius=1, color=(0, 0, 255), thickness=-1)
        robot_traj_img = cv2.circle(robot_traj_img, (1602-30,1210-26), radius=1, color=(0, 0, 255), thickness=-1)
        cv2.namedWindow("Trajectories",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Trajectories", int(1602/2), int(1210/2))
        cv2.imshow("Trajectories", robot_traj_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/prova.jpg",robot_traj_img)
        


