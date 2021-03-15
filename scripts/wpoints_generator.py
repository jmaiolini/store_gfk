#!/usr/bin/env python

import rospy
import sys
import argparse
from std_msgs.msg import String

import utils

class generator:

    def __init__(self, filename):
        self.sub = rospy.Subscriber("chatter",String,self.callback)
        print(filename)
        self.trajectory = utils.load_trajectory(filename)


    def callback(self, data):
        print(data.data)


def main():
    parser = argparse.ArgumentParser(description="ws_generator args.")
    # parser.add_argument("--filename",
    #                     help="Store filename (required)")
    parser.add_argument("filename", help="file to open",type=str)
    if len(sys.argv) != 2:    
        sys.exit('Wrong args. Add -h to see helper')
    args = parser.parse_args()
    obc = generator(args.filename)
    rospy.init_node('wpoints_generator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
