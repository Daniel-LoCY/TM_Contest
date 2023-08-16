#!/usr/bin/env python3

import rospy
from util import *

rospy.init_node('pub_static_tf')

pub_tf_static(0, 10/1000, 0, 0, 0, 0, "tool0", "tool_target")

pub_tf_static(0, 10/1000, 0, 0, 0, 0, "tool0", "tool_target_cotton")

# pub_tf_static_orientation(0.03201071899591838, 0.002545741872244266, -0.0037801534977136213,
#               0.044603988452040444, -0.00009355479851280296, 0.0002815365449583444,  0.9990047031708096,
#               "depth_camera_link", "rgb_camera_link")

rospy.spin()