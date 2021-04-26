#!/usr/bin/env python
import math

#An helper class containing all the parameters regarding cameras mounted on the robot
#Attention: data must match those in the camera description (~/catkin_ws/src/store_gfk/robot_description/custom/cameras)
#hardcoded
class CamerasParams:

  # def __init__(self):
  w = 3840 #image width
  h = 2160 #image height
  hfov = float(60*math.pi/180) # must be transformed in rads
  vfov = 2*math.atan(math.tan(hfov/2)*w/h) #not available on ROS
  base_height = 0.195 #from pal model
  camera_support_height = 1.6 
  top_camera_height = 2*(camera_support_height + base_height)/3
  bottom_camera_height = 1*(camera_support_height + base_height)/3



  # @staticmethod
  # def set_top_camera_height(h):
  #   CamerasParams.top_camera_height = h
  # @staticmethod
  # def set_bottom_camera_height(h):
  #   CamerasParams.bottom_camera_height = h
  # @staticmethod
  # def get_hfov():
  #   return CamerasParams.hfov
  # @staticmethod  
  # def get_vfov():
  #   return CamerasParams.vfov 
  # @staticmethod 
  # def get_top_camera_h():
  #   return CamerasParams.top_camera_height 
  # @staticmethod 
  # def get_bottom_camera_h():
  #   return CamerasParams.bottom_camera_height


    
