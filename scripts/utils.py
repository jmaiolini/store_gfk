#!/usr/bin/env python

import json
import sys
import os.path
import rospkg



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

    #TODO need also pixels and map meter sizes to calculate ratio. For now hardcoded
    x_off_px = 22
    y_off_px = 24
    store_width_px = 1602 
    store_height_px = 1210 
    store_width_m = 42.5 
    store_height_m = 32.4 
    
    [x_off, y_off] = traj_meter_shift(x_off_px, y_off_px, store_width_px, store_height_px,store_width_m, store_height_m)

    for coord in store_waypoints: #dict type
        new_coord = tranform_position( coord['x'], coord['y'], x_off, y_off, store_width_m, store_height_m )S
        robot_waypoints.append( new_coord )

    print(robot_waypoints)
    return robot_waypoints
    
def tranform_position(x_traj, y_traj, x_off, y_off, store_width, store_height): 
    
    x_traj += x_off
    y_traj += y_off
    
    x_map = x_traj
    y_map = store_height - y_traj
    
    if x_map >= store_width or y_map >= store_height:
        print('Trajectory out of store map. Aborting')
        sys.exit(1)

    if x_map <= 0 or y_map <= 0:
        print('Negative position. Aborting')
        sys.exit(1)


    return ( x_map, y_map )

def traj_meter_shift(x,y,w_px,h_px,w_m,h_m): #calculates shift between trajectory and map origins
    
    x_off = x*w_m/w_px
    y_off = y*h_m/h_px

    return round(x_off,3),round(y_off,3)


