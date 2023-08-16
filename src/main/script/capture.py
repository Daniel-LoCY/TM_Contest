#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_1080P,
                    depth_mode=pyk4a.DepthMode.NFOV_2X2BINNED))

k4a.start()


def main():
    rospy.init_node('camera')
    bridge = CvBridge()
    pub_color = rospy.Publisher('/camera/color', Image, queue_size=10)
    pub_ir = rospy.Publisher('/camera/ir', Image, queue_size=10)
    pub_depth = rospy.Publisher('/camera/depth', Image, queue_size=10)

    while not rospy.is_shutdown():
        capture = k4a.get_capture()

        color = capture.color
        color = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
        # cv2.imwrite('color.png', color)
        pub_color.publish(bridge.cv2_to_imgmsg(color, "passthrough"))

        ir = capture.transformed_ir
        # cv2.imwrite('ir.png', colorize(ir, (None, 5000), cv2.COLORMAP_BONE))
        pub_ir.publish(bridge.cv2_to_imgmsg(ir, "passthrough"))

        depth = capture.transformed_depth
        pub_depth.publish(bridge.cv2_to_imgmsg(depth, "passthrough"))

        # cv2.circle(color, (800, 350), 5, (0, 0, 255), -1)
        # cv2.circle(color, (1000, 350), 5, (0, 0, 255), -1)

        # cv2.circle(ir, (800+4, 350+2), 5, (0, 0, 255), -1)
        # cv2.circle(ir, (1000+2, 350+1), 5, (0, 0, 255), -1)

        # cv2.imshow('color', color)
        # cv2.imshow('ir', colorize(ir, (None, 5000), cv2.COLORMAP_BONE))
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

if __name__ == '__main__':
    main()