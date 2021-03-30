#!/usr/bin/env python

import rospy
import sys
import rospkg
import os.path
import cv2
import numpy as np

class mapGenerator():

  def __init__(self):
    a = 3
  
  def run(self):
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('store_gfk') + '/construct_map/edeka_blender.png'
    img = cv2.imread(filepath,1)

    laplacian = cv2.Laplacian(img,cv2.CV_64F)
    sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)
    sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)
    gray_img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
 
 

    crop_img = gray_img[92:img.shape[0]-70,541:gray_img.shape[1]-187]
    final_img = crop_img
    for y in range(0, crop_img.shape[0]):
      for x in range(0, crop_img.shape[1]):
        # threshold the pixel
        if crop_img[y, x] >= 120:
          final_img[y, x] = 255 
        else:
          final_img[y,x]= 0

    self.create_map(final_img)

    cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/construct_map/edeka_mmap.pgm",crop_img)
    self.show_and_wait_key("crop_img", crop_img)
    # self.show_and_wait_key("Image", img)
    # self.show_and_wait_key("crop_img", crop_img)
    # self.show_and_wait_key("gray_img", gray_img)


    cv2.destroyAllWindows()
    
    # cv2.imwrite("/home/majo/catkin_ws/src/store_gfk/prova.jpg",robot_stops_img)

  def show_and_wait_key(self,window_name,img):
    cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, img.shape[1]/2, img.shape[0]/2)
    cv2.imshow(window_name, img)
    cv2.waitKey(0)

  def create_map(self,img):
    map_img = np.zeros([1984,1984,1],dtype=np.uint8)

    real_map_img = cv2.imread('/home/majo/catkin_ws/src/store_gfk/maps/edeka/map/map.pgm',cv2.IMREAD_GRAYSCALE)
    real_map_img2 = cv2.imread('/home/majo/catkin_ws/src/store_gfk/maps/edeka/map/submap_0.pgm',cv2.IMREAD_GRAYSCALE)
    self.show_and_wait_key("real_map_img", real_map_img)
    self.show_and_wait_key("real_map_img2", real_map_img2)
    print(real_map_img.shape)
    print(real_map_img2.shape)
    histg = cv2.calcHist([real_map_img],[0],None,[256],[0,256]) 
    
    for i in range(0,len(histg)):
      if histg[i] > 0:
        print('Ce ne sono', histg[i], 'a ', i)


def main():
  #works only for Store B
  rospy.init_node('map_generator', anonymous=True)
  map = mapGenerator()
  map.run()   
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down map_generator")



    

if __name__ == '__main__':
    main()