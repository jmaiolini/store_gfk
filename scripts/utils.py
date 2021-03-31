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

    def correct_trajectory(self,traj, initial_pose):
        corr_traj = list()
        x,y,Y = initial_pose.split() 
        
        for pose in traj: #not acting on Yaw
            corr_pose = (pose[0]-float(x), pose[1]-float(y))
            corr_traj.append(corr_pose)
        print(corr_traj)
        return corr_traj
        
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

    def is_good_goal(self, store_map, goal):
        return store_map[goal[0]][goal[1]] == 0

    def find_closest_goal(self, store_map, curr_goal,shift, desired_shift): #shift in meters
        new_goal = curr_goal
        feasible_found = False
        kernel_size = 1
        while (not feasible_found):
            new_goal = self.find_feasible(store_map, curr_goal, kernel_size)
            if new_goal == (0,0):
                kernel_size+=2
            else:
                feasible_found = True #eventually will find a feasible on the boundaries
        
        #place the new point away from boundaries (TODO: make them to be aligned with rectangle)
        if shift:
            new_goal = self.shift_goal(curr_goal,new_goal,desired_shift)

        return new_goal

    def find_feasible(self, store_map, goal, kernel_size):
        for i in range(goal[0]-kernel_size,goal[0]+kernel_size):
            if i in range(goal[0]-kernel_size+2,goal[0]+kernel_size-2):
                pass
            for j in range(goal[1]-kernel_size,goal[1]+kernel_size):
                if store_map[i][j] == 255:
                    self.check_neighborhood(store_map,i,j)
                    return (i,j)
        return (0,0)

    def check_neighborhood(self,store_map,i,j):
        #TODO check if we are in vertical or horizontal edge
        k_size = 7
        sobelx = cv2.Sobel(store_map[j-k_size:j+k_size,i-k_size:i+k_size],cv2.CV_64F,1,0,ksize=k_size)
        sobely = cv2.Sobel(store_map[j-k_size:j+k_size,i-k_size:i+k_size],cv2.CV_64F,0,1,ksize=k_size)
        sobelx_full = cv2.Sobel(store_map,cv2.CV_64F,1,0,ksize=3)
        sobely_full = cv2.Sobel(store_map,cv2.CV_64F,0,1,ksize=3)
        cv2.namedWindow("sobelx",cv2.WINDOW_NORMAL)
        cv2.imshow("sobelx",sobelx_full)
        cv2.namedWindow("sobely",cv2.WINDOW_NORMAL)
        cv2.imshow("sobely",sobely_full)
        cv2.namedWindow("image",cv2.WINDOW_NORMAL)
        cv2.imshow("image",store_map)
        cv2.waitKey(0)

    def shift_goal(self,goal,new_goal,shift):
        shift = self.meters2pixels(shift,0)
        
        #calculate direction 
        delta_x = goal[0] - new_goal[0]
        delta_y = goal[1] - new_goal[1]
        
        shifted_x = new_goal[0] - cmp(delta_x,0) * int(shift[0])
        shifted_y = new_goal[1] - cmp(delta_y,0) * int(shift[0])
        
        return (shifted_x,shifted_y)
        

      

    def get_ratios(self):
        return self.px_m_ratio_x, self.px_m_ratio_y
    
    def get_goal_tollerance_r(self):
        return self.goal_tollerance



