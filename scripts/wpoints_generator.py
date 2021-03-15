#!/usr/bin/env python

import rospy
import sys
import argparse
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import utils

class generator:

    def __init__(self, filename):
        self.sub = rospy.Subscriber("chatter",String,self.callback)

        #transform and load current trajectory
        self.full_trajectory = utils.load_trajectory(filename)
        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()

        for pose in full_trajectory:
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = pose[0]
            self.goal.target_pose.pose.position.y = pose[1]
            self.goal.target_pose.pose.position.z = 0.0
            self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end 

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print(self.client.get_result())



    def callback(self, data):
        print(data.data)



def main():
    
    args = rospy.myargv()
    print(args[1])
    if len(args) != 2:
        utils.print_usage()
    rospy.init_node('wpoints_generator', anonymous=True)

    obc = generator(args[1])
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
