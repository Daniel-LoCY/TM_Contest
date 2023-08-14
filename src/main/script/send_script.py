#!/usr/bin/env python3

import rospy
from tm_msgs.srv import *
from tm_msgs.msg import *
from util import *

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
    '''
    SetIORequest_()
    : module(0)
    , type(0)
    , pin(0)
    , state(0.0)  {
    }
    '''
    try:
        tm_send_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)
        if cmd == True:
            resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_ON)
        elif cmd == False:
            resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_OFF)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def tm_ask_item_client():
    rospy.wait_for_service('tm_driver/ask_item')
    try:
        tm_ask_item = rospy.ServiceProxy('tm_driver/ask_item', AskItem)
        req = AskItemRequest()
        req.id = "demo"
        req.item = "HandCamera_Value"
        req.wait_time = 1
        resp1 = tm_ask_item(req)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def radius2degree(radius: tuple()):
    degree = []
    for i in range(len(radius)):
        degree.append(radius[i] * 180 / 3.1415926)
    return degree

def feedback_callback(msg: FeedbackState): # 手臂姿態
    return
    print(msg, '\n')
    # calculate position
    print(f"x: {msg.tool_pose[0]*1000}")
    print(f"y: {msg.tool_pose[1]*1000}")
    print(f"z: {msg.tool_pose[2]*1000}")
    print(f"rx: {msg.tool_pose[3]*180/3.1415926}")
    print(f"ry: {msg.tool_pose[4]*180/3.1415926}")
    print(f"rz: {msg.tool_pose[5]*180/3.1415926}")
    print()

    # rx, ry, rz transform to radius
    rx = msg.tool_pose[3]
    ry = msg.tool_pose[4]
    rz = msg.tool_pose[5]
    print(f"rx: {rx}")
    print(f"ry: {ry}")
    print(f"rz: {rz}")


sub = rospy.Subscriber('feedback_states', FeedbackState, feedback_callback)

delay = 5
z_low = 26 # z軸手臂底座夾取最低點 mm
z_low_floor = -280 # z軸機台放置平面夾取最低點 mm
speed = 200
m = 100

def main(x, y):


    tm_send_gripper_client(True)

    # rospy.sleep(delay)

    cmd = f"PTP(\"CPP\",{x},  {y}, 30,200,0,180,{speed},{m},0,false,0,2,4)"
    # cmd = f"Move_PTP(\"CPP\",0,0,0,20,0,0,{speed},{m},0,false)"
    pose = get_tf("base", "tool_target")
    print(pose)
    cmd = f"PTP(\"CPP\",{pose.translation.x},  {pose.translation.y}, {pose.translation.z}, {pose.rotation.x}, {pose.rotation.y}, {pose.rotation.z}, {speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)
    # print(resp)

if __name__ == '__main__':
    rospy.init_node('send_script')
    main(152, 380)