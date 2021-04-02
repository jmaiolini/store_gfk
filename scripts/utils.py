#!/usr/bin/env python

import json
import sys
import os.path
import rospy
import rospkg
import cv2
import numpy as np

class Utils:

    def __init__(self, filename='', map_source=1): #to put in config eventually

        self.filename = filename
        self.map_source = map_source

        self.map_image = []

        self.store_width = 0.0
        self.store_height = 0.0
        self.img_width = 0
        self.img_height = 0
        self.m_px_ratio_x = 0.0
        self.m_px_ratio_y = 0.0
        self.maps_x_ratio = 0.0 #set only if map_source is blender
        self.maps_y_ratio = 0.0 #set only if map_source is blender

        self.set_map_params() #with harcoded dimensions (TODO parametrize them based on store in the future)

        #trajectory image params
        # self.planimetry_width = 42.5
        # self.planimetry_height = 32.4
        # self.img_width = 1602
        # self.img_height = 1210


        #Image pixels and planimetry meters do not check in trajectory image. 
        # For now this difference is hardcoded below
        # self.m_px_ratio_x = self.planimetry_width / (self.img_width-22-30)
        # self.m_px_ratio_y = self.planimetry_height / (self.img_height-24-26)
    
        # self.store_width = self.img_width * self.m_px_ratio_x #42.5 
        # self.store_height = self.img_height * self.m_px_ratio_y #32.4 

        # #blender generated image params        
        # self.blender_store_width = 45.0
        # self.blender_store_height = 34.6
        # self.blender_img_width = 1192
        # self.blender_img_height = 918

        # #TODO (for each store) proportions. Given a goal x,y in the trajectory image
        # #we have to transform it for the blender map
        # self.maps_x_ratio = self.blender_store_width / self.store_width 
        # self.maps_y_ratio = self.blender_store_height / self.store_height

        # #OK BUT :TODO RETRIEVE IT FROM MAP YAML (0.037721614)
        # self.blender_px_m_ratio_x = self.blender_store_width / (self.blender_img_width)
        # self.blender_px_m_ratio_y = self.blender_store_height / (self.blender_img_height)




    @staticmethod
    def print_usage(exit_code=0):
        print '''Usage: rosrun store_gfk wpoints_generator.py <store_name> <traj_num> 

        - <store_name> the store name you want to inspecting under 
                       trajectories/ directory

        - <traj_num> number of the trajectory under trajectories/<store_name> 
                     you want to perform.
          Assumption: trajectories are saved as <store_name>_<traj_num>.json

        - <map_source> put 0 to set wpoints from trajectory map (output of ANN)
                       put 1 to set wpoints from blender map
          
        Example: rosrun store_gfk wpoints_generator.py edeka 1 1
        '''

        sys.exit(exit_code)

    def set_map_params(self):
        #trajectory image params
        traj_planimetry_width = 42.5
        traj_planimetry_height = 32.4
        traj_img_width = 1602
        traj_img_height = 1210

        #blender generated image params        
        blender_store_width = 45.0
        blender_store_height = 34.6
        blender_img_width = 1192
        blender_img_height = 918

        #correct trajectory dimensions
        traj_m_px_ratio_x = traj_planimetry_width / (traj_img_width-22-30) #have to correct them
        traj_m_px_ratio_y = traj_planimetry_height / (traj_img_height-24-26)
        traj_store_width = traj_img_width * traj_m_px_ratio_x #42.5
        traj_store_height = traj_img_height * traj_m_px_ratio_y #32.4 

        if self.map_source == 0: #setting params based on trajectory
            self.store_width = traj_store_width
            self.store_height = traj_store_height
            self.img_width = traj_img_width
            self.img_height = traj_img_height
            self.m_px_ratio_x = traj_m_px_ratio_x
            self.m_px_ratio_y = traj_m_px_ratio_y

        elif self.map_source == 1: #setting params based on blender
            self.store_width = blender_store_width
            self.store_height = blender_store_height
            self.img_width = blender_img_width
            self.img_height = blender_img_height
            self.m_px_ratio_x = blender_store_width / blender_img_width #TODO retrieve it from map yaml
            self.m_px_ratio_y = blender_store_height / blender_img_height #TODO retrieve it from map yaml

            #TODO (for each store) proportions. Given a goal x,y in the trajectory image
            #we have to transform it for the blender map
            self.maps_x_ratio = blender_store_width / traj_store_width 
            self.maps_y_ratio = blender_store_height / traj_store_height
            

        else:
            rospy.signal_shutdown("Wrong map source param!")

    def load_trajectory(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk') + '/trajectories/' + self.filename

    
        if os.path.exists(filepath):
            with open(filepath) as f:
                data = json.load(f)
        else:
            rospy.signal_shutdown('Could not find the trajectory file.\nExiting.')

            
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
            rospy.signal_shutdown('Trajectory out of store map. Aborting')

        if x_map <= 0 or y_map <= 0:
            rospy.signal_shutdown('Negative position. Aborting')

        return ( x_map, y_map )


    def pixels2meters(self,x_px,y_px):
        x_m = x_px * self.m_px_ratio_x
        y_m = y_px * self.m_px_ratio_y

        return x_m,y_m

    def meters2pixels(self,x_m,y_m):
        x_px = int(x_m / self.m_px_ratio_x)
        y_px = int(y_m / self.m_px_ratio_y)

        return x_px,y_px

    #changes coordinates of a point in meter from image to map coordinates
    def image2map(self,p_x,p_y):
        x_map = p_x * self.m_px_ratio_x
        y_map = self.store_height - p_y * self.m_px_ratio_y

        return x_map,y_map
    #changes coordinates of a point in pixels from map to image coordinates
    def map2image(self,p_x,p_y):
        x_img = int(p_x / self.m_px_ratio_x)
        y_img = self.img_height - int(p_y / self.m_px_ratio_y)

        return x_img,y_img

    # def map2image_blender(self,p_x,p_y):
    #     x_img = int(p_x / self.blender_px_m_ratio_x)
    #     y_img = self.blender_img_height - int(p_y / self.blender_px_m_ratio_y)

    #     return x_img,y_img

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

    def shift_goal(self,waypoint,map_shift):
        x,y,Y = map_shift.split() 
        corr_pose = (waypoint[0]+float(x), waypoint[1]+float(y))

        return corr_pose

    def is_good_goal(self, goal): #TODO parametrize based on size of robot base
        ksize = 10 #for each side (based on robot radius)
        x = self.maps_x_ratio * goal[0]
        y = self.maps_y_ratio * goal[1]
        i,j = self.map2image(x,y)
        patch = self.map_image[j-ksize:j+ksize,i-ksize:i+ksize]

        return np.average(patch) == 255

    def find_closest_goal(self, store_map, curr_goal):
        new_goal = curr_goal
        feasible_found = False
        kernel_size = 1

        while (not feasible_found):
            new_goal = self.find_feasible(store_map, curr_goal, kernel_size)
            if new_goal == (0,0):
                kernel_size+=2
                print("kernel size now ", kernel_size)
            else:
                feasible_found = True #eventually will find a feasible on the boundaries
        
            if kernel_size > 30:
                break

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
        ksize = 20
        left = store_map[j-ksize:j+ksize,i-ksize:i]
        if np.average(left) == 255:
            print("MOVING TO THE LEFT")
            return 1,i-ksize,j
        right = store_map[j-ksize:j+ksize,i: i+ksize]
        if np.average(right) == 255:
            print("MOVING TO THE RIGHT")
            return 1,i+ksize,j
        top = store_map[j-ksize:j, i-ksize:i+ksize]
        if np.average(top) == 255:
            print("MOVING TO THE TOP")
            return 1,i,j-ksize
        bottom = store_map[j:j+ksize, i-ksize:i+ksize]
        if np.average(bottom) == 255:
            print("MOVING TO THE BOTTOM")
            return 1,i,j+ksize
     
        return 0,0,0

    def get_ratios(self):
        return self.m_px_ratio_x, self.m_px_ratio_y

    def set_map_image(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk')
        if self.map_source == 0:
            filepath += '/trajectories/' + self.filename
            filename = os.path.splitext(filepath)[0]+'.jpg'
            self.map_image = cv2.imread(filename)
        elif self.map_source == 1:
            filename = rospack.get_path('store_gfk') + '/maps/edeka/trial/map.pgm'
            self.map_image = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)

    def get_map_image(self):
        return self.map_image


    def show_img_and_wait_key(self,window_name,img):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
        cv2.imshow(window_name, img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)



