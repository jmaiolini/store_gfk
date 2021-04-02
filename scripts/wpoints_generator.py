#!/usr/bin/env python

import rospy
import sys
import argparse
import actionlib
import time 
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

        #load current trajectory and tranform it
        self.full_trajectory = self.utils.load_trajectory()
        self.utils.set_map_image() #to check if the given goal is good 
        
        if debug_mode:
            counter = 0
            for waypoint in self.full_trajectory:
                if self.utils.is_good_goal(waypoint):
                    print("Goal ", counter, "is good.")
                else:
                    print("Goal ", counter, "is bad.")
                counter = counter + 1
            print("Check on the image")
            visual_tests.show_good_bad_points(self.full_trajectory, int(map_source))
            visual_tests.check_traj_correspondences(self.full_trajectory, filename, int(map_source) )
            visual_tests.check_feasibility()

            sys.exit(0)
        
        # else:
        #     #this is done to transform goal from map to robot pose and applying the map initial shift
        #     #alternatively I think it cloud be done with TF
        #     self.full_trajectory = self.utils.correct_trajectory(self.full_trajectory, rospy.get_param("map_shift")) 
            


        #ROS interface
        self.map_shift = rospy.get_param("map_shift")
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

            if self.utils.is_good_goal(waypoint):
                print("Feasible waypoint, proceeding")
                shifted_x, shifted_y = self.utils.shift_goal(waypoint,self.map_shift)
            else:
                print("NOT Feasible waypoint, changing it")
                # print("Waypoint n ", counter, "at", waypoint[0], waypoint[1], "not feasible")
                # print("Or in pixels ", self.utils.meters2pixels(waypoint[0], waypoint[1]))
                i,j = self.utils.meters2pixels(waypoint[0], waypoint[1])
              
                new_point = self.utils.find_closest_goal(self.utils.get_map_image(), (i,j))
               
                feasible_x,feasible_y = self.utils.pixels2meters(new_point[0],new_point[1])
                
                shifted_x, shifted_y = self.utils.shift_goal((feasible_x,feasible_y),self.map_shift)
                
                # print("NEW Waypoint n ", counter, "at",new_x,new_y)
                # print("Or in pixels ", i,j)

            if self.send_wp(shifted_x, shifted_y):
                print 'Goal Reached! Moving to the next if any'
                

            counter = counter +1
            time.sleep(2.0)

        
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
    
    robot_navigation = trajectoryGenerator(args, False)
    robot_navigation.start()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
