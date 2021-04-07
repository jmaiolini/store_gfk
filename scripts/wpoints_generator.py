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

    def __init__(self, args, debug_mode):

        filename = args[1] + '/' + args[1] + '_' + args[2] + '.json'

        # choose between 
        map_source = args[3]

        self.utils = Utils(filename, int(map_source))

        #load current trajectory and tranform it. full_trajectory is 
        # in the correct reference for the robot 
        self.full_trajectory = self.utils.load_trajectory()

        self.map_shift = rospy.get_param("map_shift")
        self.robot_radius = rospy.get_param("/move_base/global_costmap/robot_radius")
        self.patch_sz = 30 #based on the robot radius (must be divisible by 2)
        
        px_robot_radius = self.utils.meters2pixels(self.robot_radius,0)[0]

        self.goal_cnt = 0
        
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
            goal_cnt = 0
            for position in self.full_trajectory:
                i,j = self.utils.map2image(position[0],position[1])
                new_img_pt = self.utils.find_feasible_point(img,(i,j),self.patch_sz,goal_cnt)
                img = visual_tests.draw_patch(img,new_img_pt,self.patch_sz,(0,255,0))
                img = visual_tests.draw_robot(img,new_img_pt,px_robot_radius,(255,0,0))
                img = visual_tests.draw_arrow(img,new_img_pt,(new_img_pt[0]+25,new_img_pt[1]),(0,255,0))
                if( i!=new_img_pt[0] or j != new_img_pt[1] ):
                    img = visual_tests.draw_patch(img,(i,j),self.patch_sz,(0,0,255))
                    img = visual_tests.draw_robot(img,(i,j),px_robot_radius,(255,0,0))
                    img = visual_tests.draw_arrow(img,(i,j),(i+25,j),(0,0,255))

                goal_cnt = goal_cnt + 1

            self.utils.show_img_and_wait_key("Patches",img)
            self.utils.save_image("modified_trajectory.jpg", img)
            # visual_tests.show_good_bad_points(self.full_trajectory, int(map_source))
            # visual_tests.check_traj_correspondences(self.full_trajectory, filename, int(map_source) ) #TO REVIEW (occhio a quando moltiplico i ratio, da togliere)
            # visual_tests.check_feasibility() #stessa cosa della funzione sopra

            sys.exit(0)
    
        #ROS interface
        #subscribers
        self.laer_scan_sub = rospy.Subscriber("/scan",LaserScan, self.laser_cb)
        self.laser_msg = LaserScan()

        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = self.client.wait_for_server()
        if not wait:
            rospy.signal_shutdown("Could not find move_base Action Server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end


    def run(self):

        self.utils.set_map_image() #to check if the given goal is good 
        map_image = self.utils.get_map_image()
        img = cv2.cvtColor(map_image,cv2.COLOR_GRAY2BGR)

        for position in self.full_trajectory:
            #this double transformation is required
            i,j = self.utils.map2image(position[0],position[1])
            new_img_pt = self.utils.find_feasible_point(img,(i,j),self.patch_sz)
            new_goal_x, new_goal_y = self.utils.image2map(new_img_pt[0],new_img_pt[1])
            x, y = self.utils.shift_goal((new_goal_x,new_goal_y),self.map_shift)

            self.send_waypoint(x, y)
            self.goal_cnt = self.goal_cnt + 1
                
            time.sleep(1.0)
            # self.align_robot()


    def send_waypoint(self, x, y):

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        #send the current goal
        self.client.send_goal(self.goal,done_cb=self.done_cb,feedback_cb=self.feedback_cb) #it is possible to add the others callbacks directly from here
        
        result = self.client.wait_for_result() #blocking execution until some result comes

        if not result:
            rospy.logerr("Something went wrong with the waypoint number: !", self.goal_cnt)
            rospy.signal_shutdown("Something went wrong with the waypoint number: !", self.goal_cnt)
        else:
            return self.client.get_result()
        

    def feedback_cb(self,feedback): #returns the robot position
        # rospy.loginfo("Feedback:%s" % str(feedback))
        a = 3

    def done_cb(self, status, result):
        #status gives info on preemptions etc on the goal
        print("status: ", status)
        print("result:" ,result)

    def laser_cb(self,data):
        self.laser_msg = data

def main():
    
    args = rospy.myargv()
    
    if len(args) != 4:
        Utils.print_usage(1)
    rospy.init_node('wpoints_generator', anonymous=True)
    
    robot_navigation = trajectoryGenerator(args, True)
    robot_navigation.run()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
