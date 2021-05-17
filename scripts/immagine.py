#!/usr/bin/env python
import os
import json
import cv2

traj_planimetry_width = 42.5
traj_planimetry_height = 32.4
traj_img_width = 1602
traj_img_height = 1210


blender_store_width = 45.0
blender_store_height = 34.6
blender_img_width = 1192
blender_img_height = 918

traj_m_px_ratio_x = traj_planimetry_width / (traj_img_width-22-30) #have to correct them
traj_m_px_ratio_y = traj_planimetry_height / (traj_img_height-24-26)
traj_store_width = traj_img_width * traj_m_px_ratio_x #42.5
traj_store_height = traj_img_height * traj_m_px_ratio_y #32.4 

store_width = blender_store_width
store_height = blender_store_height
img_width = blender_img_width
img_height = blender_img_height
m_px_ratio_x = blender_store_width / blender_img_width #TODO retrieve it from map yaml
m_px_ratio_y = blender_store_height / blender_img_height #TODO retrieve it from map yaml

maps_x_ratio = blender_store_width / traj_store_width 
maps_y_ratio = blender_store_height / traj_store_height


def main():

    filepath = 'home/majo/catkin_ws/src/store_gfk/trajectories/edeka/edeka_2.json'
    img = read_image('home/majo/catkin_ws/src/store_gfk/trajectories/edeka/map_ref.png')

    if os.path.exists(filepath):
        with open(filepath) as f:
            data = json.load(f)

    trajectory_meter = list()
        
    for coord in data["points"]: #dict type
        new_coord = tranform_position( coord['x'], coord['y'])
        trajectory_meter.append( new_coord )

    for wpoint in trajectory_meter:
        draw_point(img,wpoint)

    show_img_and_wait_key("img",img)

def tranform_position(x_traj, y_traj):

    x = maps_x_ratio * x_traj
    y = maps_y_ratio * y_traj

    x_map, y_map = map2image_meters(x,y) 

    return ( x_map, y_map )

def map2image_meters(p_x,p_y):
    x = p_x
    y = store_height - p_y
    
    return x,y

def show_img_and_wait_key(window_name,img):
    cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, int(1602/2), int(1210/2))
    cv2.imshow(window_name, img)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)

def read_image(filename, coding = 1):
        return cv2.imread(filename,1)

def save_image(filename,image):
    cv2.imwrite(filename,image)

def gray2bgr(image):
    return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

def bgr2gray(image):
    return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

def draw_point(img,pt,color=(0,0,255),thickness=3):
    return cv2.circle(img,pt,thickness,color,-1)


if __name__ == '__main__':
    main()

