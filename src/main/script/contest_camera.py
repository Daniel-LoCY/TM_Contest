#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import pyk4a
from pyk4a import Config, PyK4A

k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_1080P,
                    depth_mode=pyk4a.DepthMode.NFOV_UNBINNED))

k4a.start()


def main():
    rospy.init_node('listener')
    bridge = CvBridge()
    pub = rospy.Publisher('/camera/color', Image, queue_size=10)

    while not rospy.is_shutdown():
        frame = k4a.get_capture().color
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        pub.publish(bridge.cv2_to_imgmsg(frame, "passthrough"))
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()