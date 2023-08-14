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
    rospy.init_node('camera')
    bridge = CvBridge()
    pub_color = rospy.Publisher('/camera/color', Image, queue_size=10)
    pub_ir = rospy.Publisher('/camera/ir', Image, queue_size=10)
    pub_depth = rospy.Publisher('/camera/depth', Image, queue_size=10)

    while not rospy.is_shutdown():
        capture = k4a.get_capture()
        frame = capture.color
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        pub_color.publish(bridge.cv2_to_imgmsg(frame, "passthrough"))

        pub_ir.publish(bridge.cv2_to_imgmsg(capture.ir, "passthrough"))
        pub_depth.publish(bridge.cv2_to_imgmsg(capture.depth, "passthrough"))
        
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()