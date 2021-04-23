#!/usr/bin/env python
import math

#An helper class containing all the parameters regarding cameras mounted on the robot
#Attention: data must match those in the camera description (~/catkin_ws/src/store_gfk/robot_description/custom/cameras)

@staticmethod
class CameraParams:

  def __init__(self):
    self.w = 3840 #image width
    self.h = 2160 #image height
    self.hfov = 60 # must be transformed in rads
    self.vfov = 2*math.atan(math.tan(self.hfov/2)*w/h) #not available on ROS
  
  def get_hfov(self):
    return self.hfov
   
   def get_vfov(self):
    return self.vfov 


    
