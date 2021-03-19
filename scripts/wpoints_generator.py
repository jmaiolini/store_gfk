#!/usr/bin/env python

import rospy
import sys
import argparse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from utils import Utils
import visual_tests

class trajectoryGenerator:

    def __init__(self, filename, debug_mode=False):

        utils = Utils()
        #transform and load current trajectory
        self.full_trajectory = utils.load_trajectory(filename)

        if debug_mode:
            x_ratio, y_ratio = utils.get_ratios()
            radious = utils.get_goal_tollerance_r()
            visual_tests.check_traj_correspondences(self.full_trajectory, filename, x_ratio, y_ratio)
            # visual_tests.check_radious(radious)
        
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

        # for waypoint in self.full_trajectory:
            # send_waypoint(self.goal, waypoint[0], waypoint[1])
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.full_trajectory[0][0]
        self.goal.target_pose.pose.position.y = self.full_trajectory[0][1]


        print 'Current Goal: ( ', self.full_trajectory[0][0] , ',' , self.full_trajectory[0][1], ' )'
        
        #send the current goal
        self.client.send_goal(self.goal,feedback_cb=self.feedback_cb) #it is possible to add the others callbacks directly from here
        
        wait = self.client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print(self.client.get_result())

        # align_robot()

    # def send_waypoint(self,goal, x_waypoint, y_waypoint):
    # ...
    
    # def align_robot(self):
    # ...

    def feedback_cb(self,feedback):
        # rospy.loginfo("Feedback:%s" % str(feedback))
        print("\n")
        
    def map_cb(self,data):
        print("Getting map data.\n")

    def laser_cb(self,data):
        self.laser_msg = data.data


def main():
    
    args = rospy.myargv()
    
    if len(args) != 2:
        Utils.print_usage(1)
    rospy.init_node('wpoints_generator', anonymous=True)

    robot_navigation = trajectoryGenerator(args[1], True)
    robot_navigation.start()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
