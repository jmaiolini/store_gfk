#!/usr/bin/env python

import rospy
import sys
import argparse
import json
from std_msgs.msg import String

class generator:

    def __init__(self, filename):
        self.sub = rospy.Subscriber("chatter",String,self.callback)
        print(filename)
    def callback(self, data):
        print(data.data)


def main():
    parser = argparse.ArgumentParser(description="ws_generator args.")
    parser.add_argument("--filename",
                        help="Provide store filename (required)")

    args = parser.parse_args()

    obc = generator(args)
    rospy.init_node('wpoints_generator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down wpoints_generator")

if __name__ == '__main__':
    main()
