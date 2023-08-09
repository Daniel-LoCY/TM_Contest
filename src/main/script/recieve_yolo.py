#!/usr/bin/env python3

import rospy
from contest_msgs.msg import yolo
from cv_bridge import CvBridge

def callback(msg: yolo):
    print(msg)

def main():
    rospy.init_node('recieve_yolo')
    rospy.Subscriber('/yolo', yolo, callback)
    rospy.spin()

if __name__ == '__main__':
    main()