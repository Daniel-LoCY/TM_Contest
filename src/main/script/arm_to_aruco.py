#!/usr/bin/env python3

#移動手臂到aruco中心點，定位手臂


import rospy
from tm_msgs.srv import *
from tm_msgs.msg import *

delay = 3
z_low = 26 # z軸手臂底座夾取最低點 mm
z_low_floor = -280 # z軸機台放置平面夾取最低點 mm
speed = 200
m = 100

def tm_send_script_client(cmd: str):
    rospy.wait_for_service('tm_driver/send_script')
    try:
        tm_send_script = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        resp1 = tm_send_script("demo", cmd)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def tm_send_gripper_client(cmd: bool): # True: 夾取, False: 放開
    rospy.wait_for_service('tm_driver/set_io')
    try:
        tm_send_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)
        if cmd == True:
            resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_ON)
        elif cmd == False:
            resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_OFF)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    x = 0
    y = 500
    z = 40-2
    ready = True
    # ready = False
    if ready:
        x = x
        y = 200
        z = 100
    rospy.init_node("move_to_aruco_center")
    rospy.loginfo("move_to_aruco_center")
    tm_send_gripper_client(True)
    cmd = f"PTP(\"CPP\",{x}, {y}, {z},180,0,180,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)
    tm_send_gripper_client(True)
    if ready:
        exit()
    with open('point_at_arm.txt', 'w') as f:
        f.write(f"[{x}, {y}, {z}]\n")