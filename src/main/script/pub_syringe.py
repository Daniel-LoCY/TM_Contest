#!/usr/bin/env python3

import rospy
from util import *

rospy.init_node('pub_static_tf_syringe')

pub_tf_static(0, 15/1000, 0, 0, 0, 0, "tool0", "tool_target")
rospy.spin()