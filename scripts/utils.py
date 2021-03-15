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

    for coord in store_waypoints: #dict type
        new_coord = tranform_position( coord['x'], coord['y'] )

        robot_waypoints.append( new_coord )

    print(robot_waypoints)
    return robot_waypoints
    
def tranform_position():


    return 