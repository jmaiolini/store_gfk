#!/usr/bin/env python

import rospy
import sys
import argparse
import actionlib
import time 
import cv2
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from utils import Utils
import visual_tests

class trajectoryGenerator:

    def __init__(self, args, debug_mode=False):

        filename = args[1] + '/' + args[1] + '_' + args[2] + '.json'

        # choose between 
        map_source = args[3]

        self.utils = Utils(filename, int(map_source))

        #load current trajectory and tranform it. full_trajectory is 
        # in the correct reference for the robot 
        self.full_trajectory = self.utils.load_trajectory()

        self.map_shift = rospy.get_param("map_shift")
        self.robot_radius = rospy.get_param("/move_base/global_costmap/robot_radius")
        px_robot_radius = self.utils.meters2pixels(self.robot_radius,0)[0]
        
        if debug_mode:
            #performing all the steps to send goal but just for visualization except the map shift
            self.utils.set_map_image() #to check if the given goal is good 
            map_image = self.utils.get_map_image()
            img = cv2.cvtColor(map_image,cv2.COLOR_GRAY2BGR)

            #The drawings are the following:
            # at each waypoint of the trajectory we have a blue circle representing the robot size
            # a yellow square showing the area on which we ask if the waypoint is good or needs to be refined
            # arrow (for now only pointing at 0 degrees) to show the robot heading
            #if the point is bad, additional squares and arrows are shown

            prev_waypoint = (0,0)
            for waypoint in self.full_trajectory:
                if self.utils.is_good_goal_meters(waypoint):
                    print("GOOD GOAL, showing it as green for the direction")
                    i,j = self.utils.map2image(waypoint[0],waypoint[1])
                    img = visual_tests.draw_patch(img,(i,j),20,(0,255,255))
                    img = visual_tests.draw_robot(img,(i,j),px_robot_radius,(255,255,0))
                    img = visual_tests.draw_arrow(img,(i,j),(i+20,j),(0,255,0))

                    prev_waypoint = (i,j)
                     
                    #shift for wpoint
                else:
                    print("BAD GOAL, modifying it")
                    i,j = self.utils.map2image(waypoint[0],waypoint[1])
                    img = visual_tests.draw_patch(img,(i,j),20,(0,255,255))
                    img = visual_tests.draw_robot(img,(i,j),px_robot_radius,(255,255,0))
                    img = visual_tests.draw_arrow(img,(i,j),(i+20,j),(0,0,255))
                    new_pt = self.utils.find_closest_goal(img, (i,j)) 
                    new_pt2 = self.utils.find_closest_goal2(img, (i,j), prev_waypoint) #2nd approach (optimized)
                    # img = visual_tests.draw_patch(img,new_pt,20,(0,255,255))
                    # img = visual_tests.draw_robot(img,new_pt,px_robot_radius,(255,255,0))
                    # img = visual_tests.draw_arrow(img,new_pt,(new_pt[0]+20,new_pt[1]),(0,255,0))
                    img = visual_tests.draw_patch(img,new_pt2,20,(0,255,255))
                    img = visual_tests.draw_robot(img,new_pt2,px_robot_radius,(255,255,0))
                    img = visual_tests.draw_arrow(img,new_pt2,(new_pt2[0]+20,new_pt2[1]),(0,255,0))
                    new_pt3 = self.utils.find_closest_goal3(img, (i,j)) #3nd approach (robust)
                    img = visual_tests.draw_patch(img,new_pt3,20,(0,255,255))
                    img = visual_tests.draw_robot(img,new_pt3,px_robot_radius,(255,255,0))
                    img = visual_tests.draw_arrow(img,new_pt3,(new_pt3[0]+20,new_pt3[1]),(0,255,0))
                    prev_waypoint = new_pt2

            


            self.utils.show_img_and_wait_key("Patches",img)
            self.utils.save_image("modified_trajectory.jpg", img)
            # visual_tests.show_good_bad_points(self.full_trajectory, int(map_source))
            # visual_tests.check_traj_correspondences(self.full_trajectory, filename, int(map_source) ) #TO REVIEW (occhio a quando moltiplico i ratio, da togliere)
            # visual_tests.check_feasibility() #stessa cosa della funzione sopra

            sys.exit(0)
    

        #ROS interface
        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        
        #subscribers
        self.occupancy_grid_sub = rospy.Subscriber("/map",OccupancyGrid, self.map_cb)
        self.laer_scan_sub = rospy.Subscriber("/scan",LaserScan, self.laser_cb)
        self.laser_msg = LaserScan()


    def start(self):
        
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end

        counter = 0
        shifted_x = 0.0
        shifted_y = 0.0
        for waypoint in self.full_trajectory:

            if self.utils.is_good_goal_meters(waypoint):
                print("Feasible waypoint, proceeding")
                shifted_x, shifted_y = self.utils.shift_goal(waypoint,self.map_shift)
            else:
                print("NOT Feasible waypoint, changing it")
                i,j = self.utils.meters2pixels(waypoint[0], waypoint[1])
                new_point = self.utils.find_closest_goal(self.utils.get_map_image(), (i,j))  
                feasible_x,feasible_y = self.utils.pixels2meters(new_point[0],new_point[1])   
                shifted_x, shifted_y = self.utils.shift_goal((feasible_x,feasible_y),self.map_shift)
                

            self.trials = 0
            if self.send_wp(shifted_x, shifted_y):
                print 'Goal Reached! Moving to the next if any'
                

            counter = counter +1
            time.sleep(1.0)

        
        # align_robot()

    def send_wp(self, x_waypoint, y_waypoint):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x_waypoint
        self.goal.target_pose.pose.position.y = y_waypoint

        print 'Current Goal: ( ', x_waypoint , ',' , y_waypoint, ' )'
        #send the current goal
        self.client.send_goal(self.goal,done_cb=self.done_cb,feedback_cb=self.feedback_cb) #it is possible to add the others callbacks directly from here
        
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
        

    # def align_robot(self):
    # ...

    def feedback_cb(self,feedback): #returns the robot position
        # rospy.loginfo("Feedback:%s" % str(feedback))
        a = 3

    def done_cb(self, status, result):
        self.trials = self.trials + 1

        if (self.trials > 2):
            print("status: ", status)
            print("result:" ,result)
        
    def map_cb(self,data):
        a = 3

    def laser_cb(self,data):
        self.laser_msg = data

def main():
    
    args = rospy.myargv()
    
    if len(args) != 4:
        Utils.print_usage(1)
    rospy.init_node('wpoints_generator', anonymous=True)
    
    robot_navigation = trajectoryGenerator(args, True)
    robot_navigation.start()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
