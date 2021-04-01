#!/usr/bin/env python

import json
import sys
import os.path
import rospkg
import cv2
import numpy as np

#TODO modify them based on the store (for now only store B available)
# px_m_ratio_x = 0.026529338 #m/px WRONG
# px_m_ratio_y = 0.02677686 #m/px WRONG


class Utils:

    def __init__(self): #to put in config eventually

        self.goal_tollerance = 1

        #trajectory image params
        self.planimetry_width = 42.5
        self.planimetry_height = 32.4
        self.img_width = 1602
        self.img_height = 1210

        #blender generated image params        
        self.blender_store_width = 45.0
        self.blender_store_height = 34.6
        self.blender_img_width = 1192
        self.blender_img_height = 918

        #Image pixels and planimetry meters do not check in trajectory image. 
        # For now this difference is hardcoded below
        self.px_m_ratio_x = self.planimetry_width / (self.img_width-22-30)
        self.px_m_ratio_y = self.planimetry_height / (self.img_height-24-26)
    
        self.store_width = self.img_width * self.px_m_ratio_x #42.5 
        self.store_height = self.img_height * self.px_m_ratio_y #32.4 

        #TODO (for each store) proportions. Given a goal x,y in the trajectory image
        #we have to transform it for the blender map
        self.maps_x_ratio = self.blender_store_width / self.store_width 
        self.maps_y_ratio = self.blender_store_height / self.store_height

        #OK BUT :TODO RETRIEVE IT FROM MAP YAML (0.037721614)
        self.blender_px_m_ratio_x = self.blender_store_width / (self.blender_img_width)
        self.blender_px_m_ratio_y = self.blender_store_height / (self.blender_img_height)

    @staticmethod
    def print_usage(exit_code=0):
        print '''Usage: rosrun store_gfk wpoints_generator.py <store_name> <traj_num> 

        - <store_name> the store name you want to inspecting under 
                       trajectories/ directory

        - <traj_num> number of the trajectory under trajectories/<store_name> 
                     you want to perform.
          Assumption: trajectories are saved as <store_name>_<traj_num>.json
          
        Example: rosrun store_gfk wpoints_generator.py edeka 1 
        '''

        sys.exit(exit_code)

    def load_trajectory(self, filename):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk') + '/trajectories/' + filename

    
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

        return robot_waypoints

    def correct_trajectory(self,traj, map_shift):
        corr_traj = list()
        x,y,Y = map_shift.split() 
        
        for pose in traj: #not acting on Yaw
            corr_pose = (pose[0]+float(x), pose[1]+float(y))
            corr_traj.append(corr_pose)
        return corr_traj
        
    def tranform_position(self, x_traj, y_traj): 

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

    def map2image_blender(self,p_x,p_y):
        x_img = int(p_x / self.blender_px_m_ratio_x)
        y_img = self.blender_img_height - int(p_y / self.blender_px_m_ratio_y)

        return x_img,y_img

    #change reference (invert y axis) of a point in pixels
    def map2image_pixels(self,p_x,p_y):
        x = p_x
        y = self.img_height - p_y

        return x,y
    #change reference (invert y axis) of a point in meters
    def map2image_meters(self,p_x,p_y):
        x = p_x
        y = self.store_height - p_y
       
        return x,y

    def is_good_goal(self, store_map, goal):
        return store_map[goal[0]][goal[1]] == 0

    def find_closest_goal(self, store_map, curr_goal):
        new_goal = curr_goal
        feasible_found = False
        kernel_size = 1
        while (not feasible_found):
            new_goal = self.find_feasible(store_map, curr_goal, kernel_size)
            if new_goal == (0,0):
                kernel_size+=2
            else:
                feasible_found = True #eventually will find a feasible on the boundaries
        if kernel_size == 1:
            return curr_goal
        return new_goal

    def find_feasible(self, store_map, goal, ksize):
        for i in range(goal[0]-ksize,goal[0]+ksize):
            if i in range(goal[0]-ksize+2,goal[0]+ksize-2):
                pass
            for j in range(goal[1]-ksize,goal[1]+ksize):
                if store_map[j][i] == 255:
                    found_neigh,x,y = self.check_neighbors(store_map,i,j)
                    if found_neigh:
                        return (x,y)
        return (0,0)

    def check_neighbors(self,store_map,i,j):
        ksize = 10
        left = store_map[j-ksize:j+ksize,i-ksize:i]
        if np.average(left) == 255:
            return 1,i-ksize,j
        right = store_map[j-ksize:j+ksize,i: i+ksize]
        if np.average(right) == 255:
            return 1,i+ksize,j
        top = store_map[j-ksize:j, i-ksize:i+ksize]
        if np.average(top) == 255:
            return 1,i,j-ksize
        bottom = store_map[j:j+ksize, i-ksize:i+ksize]
        if np.average(bottom) == 255:
            return 1,i,j+ksize
     
        return 0,0,0

    def get_ratios(self):
        return self.px_m_ratio_x, self.px_m_ratio_y
    
    def get_goal_tollerance_r(self):
        return self.goal_tollerance

    def show_img_and_wait_key(self,window_name,img):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
        cv2.imshow(window_name, img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)



