#!/usr/bin/env python

import json
import sys
import os.path
import rospy
import rospkg
import cv2
import numpy as np
import math 
from shelf import Shelf
from cameras import CamerasParams

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
        self.maps_x_ratio = 0.0
        self.maps_y_ratio = 0.0 

        self.robot_pose = 0
        self.object_pose = 0

        self.set_map_params() #with harcoded dimensions (TODO parametrize them based on store in the future)

        self.shelfs = list()
        self.repulsive_areas = list()



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
        - <debug_mode> enable visualization simulating waypoints on the map
                       or send real waypoints to the robot
          
        Example: rosrun store_gfk wpoints_generator.py edeka 1 1 False
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

            self.maps_x_ratio = 1.0
            self.maps_y_ratio = 1.0

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

    ##################################
    ##SHELFS/FORBIDDEN AREAS UTILS
    ##################################

    #TODO in init
    #extracts shelfs from file and convert them into image coordinates
    def parse_shelfs(self,filepath):
        if os.path.exists(filepath):
            with open(filepath,'rb') as json_file:
                data = json.load(json_file)
        else:
            rospy.signal_shutdown('Could not find shelfs file.\nCheck file path.')
        
        for shelf in data['shelfs']:
            p1_x,p1_y = self.map2image(shelf['x'],shelf['y'])
            w,h = self.meters2pixels(shelf['w'],shelf['h'])
            img_shelf = Shelf(shelf['id'],p1_x,p1_y,shelf['z'],w,h,shelf['dir'])
            self.shelfs.append(img_shelf)
            
    #TODO in init
    def calculate_repulsive_areas(self):
    
        for shelf in self.shelfs:
            d_min = (shelf.z - CamerasParams.top_camera_height)/math.tan(CamerasParams.vfov/2)
            d_min_px = self.map2image(d_min,0)[0]
   
            repulsive_shelf = Shelf(shelf.id,shelf.x-d_min_px,shelf.y-d_min_px,0.0,shelf.w+2*d_min_px,shelf.h+2*d_min_px,shelf.dir) #basicallly enhancing the shelfs
            self.repulsive_areas.append(repulsive_shelf)

    def is_inside_a_repulsive_area(self,regions,pt):
        for region in regions:
            if region.x <= pt[0] <= region.x+region.w and region.y <= pt[1] <= region.y+region.h:
                return True       
        return False

    #since shelfs and repulsive areas share the same center and id, we get the query shelf by using the repulsive areas
    def find_query_shelf(self,goal):
        dist_to_shelf_center = 99999
        query_shelf = 0

        for rep_area in self.repulsive_areas:
            dist = self.calc_eucl_dist(rep_area.center,goal)
            if  dist < dist_to_shelf_center:
                query_shelf = rep_area
                dist_to_shelf_center = dist

        if query_shelf == 0:
            print("Something went wrong when calculating query shelf")
            sys.exit(-1)
        
        if dist_to_shelf_center > 150: #the generated wpoint is too far/wrong, thus not scanning
            return 0

        return query_shelf

    def get_repulsive_areas(self):
        return self.repulsive_areas 

    def get_shelfs(self):
        return self.shelfs

    ##################################
    ##TRAJECTORY UTILS
    ##################################

 #TODO in init instead of in wpoint generator
    def load_trajectory(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk') + '/trajectories/' + self.filename

        if os.path.exists(filepath):
            with open(filepath) as f:
                data = json.load(f)
        else:
            rospy.signal_shutdown('Could not find the trajectory file.\nCheck file path.')

            
        return self.transform_waypoints(data["points"])
 #TODO in init instead of in wpoint generator
    def transform_waypoints(self, store_waypoints):
        robot_waypoints = list()
        
        for coord in store_waypoints: #dict type
            new_coord = self.tranform_position( coord['x'], coord['y'])
            robot_waypoints.append( new_coord )

        return robot_waypoints
 #TODO in init instead of in wpoint generator
    def correct_trajectory(self,traj, map_shift):
        corr_traj = list()
        x,y,Y = map_shift.split() 
        
        for pose in traj: #not acting on Yaw
            corr_pose = (pose[0]+float(x), pose[1]+float(y))
            corr_traj.append(corr_pose)
        return corr_traj
        #TODO in init 
    def tranform_position(self, x_traj, y_traj):

        x = self.maps_x_ratio * x_traj
        y = self.maps_y_ratio * y_traj

        x_map, y_map = self.map2image_meters(x,y) 
            
        if x_map >= self.store_width or y_map >= self.store_height:
            rospy.signal_shutdown('Trajectory out of store map. Aborting')

        if x_map <= 0 or y_map <= 0:
            rospy.signal_shutdown('Negative position. Aborting')

        return ( x_map, y_map )

    ##################################
    ## COORDINATES UTILS
    ##################################

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

    ##################################
    ## OLD APPROACH FEASIBLE POINTS UTILS
    ##################################
    
    #this function returns 2 positions: the interested object on the shelf and the robot position in a free area
    def find_feasible_point(self, curr_goal, patch_sz, patch_len, iter):
        initial_patch = self.map_image[curr_goal[1]-patch_sz/2:curr_goal[1]+patch_sz/2,curr_goal[0]-patch_sz/2:curr_goal[0]+patch_sz/2]
        mean = np.average(initial_patch)
        new_x = 0
        new_y = 0
        #the waypoint is good, meaning is not on a shelf -> we modify only the object position (closest)
        if mean == 255:
            self.robot_pose = curr_goal
            shelf_found = False
            #we first try to look around for a shelf in the 4 canonical directions
            for slide_cnt in range(1,80): 
                shelf_found, x_shelf, y_shelf = self.search_closest_shelf(curr_goal, patch_len, slide_cnt)
                if shelf_found:
                    self.object_pose = (x_shelf, y_shelf)
                    break
            #if they are too far, we look for the closest point (here we capture also corners)
            if shelf_found == False: #after 100 pixel, if we do not find a small line we seek for the closest point
                for slide_cnt in range(1,200): 
                    point_found, x_shelf_pt, y_shelf_pt = self.search_closest_point( curr_goal, slide_cnt)
                    if point_found:
                        self.object_pose = (x_shelf_pt, y_shelf_pt)
                        break
            return curr_goal #already a valid point
        #here we fo the same thing, however we need to modify also the robot position
        #get a good robot position
        for slide_cnt in range(1,100): 
            good_spot_found,new_x,new_y = self.slide_patch( curr_goal, patch_sz, slide_cnt)
            if good_spot_found:
                self.robot_pose = (new_x,new_y) 
                # self.object_pose = curr_goal
                break
        shelf_found = False
         #now we look for a good object position (as above) but with the new robot position
        for slide_cnt in range(1,80): 
            shelf_found, x_shelf, y_shelf = self.search_closest_shelf(self.robot_pose, patch_len, slide_cnt)
            if shelf_found:
                self.object_pose = (x_shelf, y_shelf)
                break
        
        if shelf_found == False:
            for slide_cnt in range(1,200): 
                point_found, x_shelf_pt, y_shelf_pt = self.search_closest_point( self.robot_pose, slide_cnt)
                if point_found:
                    self.object_pose = (x_shelf_pt, y_shelf_pt)
                    break
        return new_x,new_y

    def slide_patch(self,goal, patch_sz, it):
        i = goal[0]
        j = goal[1]
        up_patch = self.map_image[j-patch_sz/2-it:j+patch_sz/2-it,i-patch_sz/2:i+patch_sz/2]
        down_patch = self.map_image[j-patch_sz/2+it:j+patch_sz/2+it,i-patch_sz/2:i+patch_sz/2]
        left_patch = self.map_image[j-patch_sz/2:j+patch_sz/2,i-patch_sz/2-it:i+patch_sz/2-it]
        right_patch = self.map_image[j-patch_sz/2:j+patch_sz/2,i-patch_sz/2+it:i+patch_sz/2+it]

        if self.is_good_spot(up_patch):
            return 1,i,j-it
        if self.is_good_spot(down_patch):
            return 1,i,j+it
        if self.is_good_spot(left_patch):
            return 1,i-it,j
        if self.is_good_spot(right_patch):
            return 1,i+it,j
        return 0,0,0

    def is_good_spot(self,patch):
        return np.average(patch) == 255

    def search_closest_shelf(self, goal, patch_len, it):
        #this searches the closest shelf with increasing line patches positions
        # -----------
        # | ------- |
        # | |     | |
        # | |  .  | |
        # | |     | |
        # | ------- |
        # -----------
        
        i = goal[0]
        j = goal[1]
        #TODO also for patch above
        if j-it > 0:
            up_line = self.map_image[j-it,i-patch_len/2:i+patch_len/2]
            if self.is_closest_shelf(up_line):
                return 1,i,j-it
        if j+it < self.map_image.shape[0]:
            down_line = self.map_image[j+it,i-patch_len/2:i+patch_len/2]
            if self.is_closest_shelf(down_line):        
                return 1,i,j+it
        if i-it > 0:
            left_line = self.map_image[j-patch_len/2:j+patch_len/2,i-it]
            if self.is_closest_shelf(left_line):
                return 1,i-it,j
        if i+it < self.map_image.shape[1]:  
            right_line = self.map_image[j-patch_len/2:j+patch_len/2,i+it]
            if self.is_closest_shelf(right_line):
                return 1,i+it,j

        return 0,0,0
    
    def search_closest_point(self, goal, it):
        #make frame around as before
        i = goal[0]
        j = goal[1]
        #TODO take the CLOSEST (euclidian) wrt the center and not the first as right now
        if j-it > 0:
            up_line = self.map_image[j-it,i-it:i+it]
            shift = 0
            for x in up_line:
                if x == 0:
                    return 1,i-it+shift,j-it
                else:
                    shift = shift + 1

        if j+it < self.map_image.shape[0]:
            down_line = self.map_image[j+it,i-it:i+it]
            shift = 0
            for x in down_line:
                if x == 0:
                    return 1,i-it+shift,j+it
                else:
                    shift = shift + 1

        if i-it > 0:
            left_line = self.map_image[j-it:j+it,i-it]
            shift = 0
            for y in left_line:
                if y == 0:
                    return 1,i-it,j-it+shift
                else:
                    shift = shift + 1

        if i+it < self.map_image.shape[1]:  
            right_line = self.map_image[j-it:j+it,i+it]
            shift = 0
            for y in right_line:
                if y == 0:
                    return 1,i+it,j-it+shift
                else:
                    shift = shift + 1

        return 0,0,0

    def is_closest_shelf(self,patch):
        return np.average(patch) == 0

    def get_ratios(self):
        return self.m_px_ratio_x, self.m_px_ratio_y

    def get_object_direction(self):
        return self.robot_pose,self.object_pose

    ##################################
    ## MAP UTILS
    ##################################

    def set_map_image(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('store_gfk')
        if self.map_source == 0:
            filepath += '/trajectories/' + self.filename #to parametrize
            filename = os.path.splitext(filepath)[0]+'.jpg'
            self.map_image = cv2.imread(filename)
        elif self.map_source == 1:
            filename = filepath + '/maps/edeka/trial/map.pgm' #to parametrize
            self.map_image = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)

    def get_map_image(self):
        return self.map_image

    ##################################
    ## OPENCV UTILS
    ##################################

    def show_img_and_wait_key(self,window_name,img):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
        cv2.imshow(window_name, img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

    def read_image(self,filename, coding = 1):
        return cv2.imread(filename,1)

    def save_image(self,filename,image):
        cv2.imwrite(filename,image)

    def gray2bgr(self,image):
        return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

    def bgr2gray(self,image):
        return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    ##################################
    ## MISCELLANEOUS UTILS
    ##################################  

    def save_pose(self,path,time,x,y,yaw):
        f = open(path, "w")
        f.write('# pose.yaml file')
        f.write('\n\n')
        f.write('capture_time: ' + str(time) + '\n')
        f.write('x: ' + str(x) + '\n')
        f.write('y: ' + str(y) + '\n')
        f.write('yaw: ' + str(yaw) )
        f.close()

    def dir_exists(self,path):
        if os.path.exists(path):
            return True
        else:
            return self.create_dir(path)

    def create_dir(self,path):
        return os.makedirs(path)

    def calc_eucl_dist(self,p1,p2):
       return math.sqrt((p2[0] - p1[0])**2 + (p1[1] - p2[1])**2)

    def pi(self):
        return math.pi




