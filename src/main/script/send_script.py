#!/usr/bin/env python3

import rospy
from tm_msgs.srv import *
from tm_msgs.msg import *

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
    print(msg, '\n')
    # calculate position
    print(f"x: {msg.tool_pose[0]*1000}")
    print(f"y: {msg.tool_pose[1]*1000}")
    print(f"z: {msg.tool_pose[2]*1000}")
    print(f"rx: {msg.tool_pose[3]*180/3.1415926}")
    print(f"ry: {msg.tool_pose[4]*180/3.1415926}")
    print(f"rz: {msg.tool_pose[5]*180/3.1415926}")
    print()

sub = rospy.Subscriber('feedback_states', FeedbackState, feedback_callback)

delay = 3
z_low = 26 # z軸手臂底座夾取最低點 mm
z_low_floor = -280 # z軸機台放置平面夾取最低點 mm
speed = 200
m = 100

def main(x, y):


    tm_send_gripper_client(False)

    rospy.sleep(delay)

    cmd = f"PTP(\"CPP\",{x}, {y}, {z_low_floor},180,0,180,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    tm_send_gripper_client(True)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",0,0,50,0,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",0,0,0,-20,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay+3)

    cmd = f"Move_PTP(\"CPP\",0,0,0,20,0,0,{speed},{m},0,false)"    
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",0,0,-50,0,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    tm_send_gripper_client(False)

    rospy.sleep(delay)

    cmd = f"PTP(\"CPP\",{x}, {y}, {z_low_floor},180,0,180,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

    exit()
    # target = (2.1879980241745374, -0.19477874185940608, 1.5178682638164982, 0.252481270105828, 1.5732107486391624, -0.16581598283268328)
    # target_degree = radius2degree(target)
    # cmd = f"PTP(\"JPP\",{target_degree[0]},{target_degree[1]},{target_degree[2]},{target_degree[3]},{target_degree[4]},{target_degree[5]},{speed},{m},0,false)"
    # resp = tm_send_script_client(cmd)

def catch_test():

    cmd = f"PTP(\"CPP\",100,300,{z_low},180,0,-135,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    cmd = f"PTP(\"CPP\",-100,250,{z_low},180,0,-135,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    tm_send_gripper_client(True)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",0,0,50,0,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",-100,-100,0,0,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    cmd = f"Move_PTP(\"CPP\",0,0,-50,0,0,0,{speed},{m},0,false)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

    tm_send_gripper_client(False)

    rospy.sleep(delay)

    cmd = f"PTP(\"CPP\",-100,250,100,180,0,-135,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

    rospy.sleep(delay)

def ready():
    tm_send_gripper_client(True)
    cmd = f"PTP(\"CPP\",391,300, {z_low_floor+100},180,0,180,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)

def test():
    x = 390
    y = 545
    z = z_low_floor+10
    cmd = f"PTP(\"CPP\",{x}, {y}, {z},180,0,180,{speed},{m},0,false,0,2,4)"
    resp = tm_send_script_client(cmd)
    tm_send_gripper_client(True)
    with open('point_at_arm.txt', 'w') as f:
        f.write(f"{x} {y} {z}\n")

if __name__ == '__main__':

    rospy.init_node('send_script')
    rospy.loginfo('Sending script')
    
    catch_test(), exit()
    # ready(), exit()
    # test(), exit()
    main(370.5, 500.5)
    # radius_set = (2.419723132731749, -0.5256350951468007, 2.37297879664646, -0.27194168727082313, 1.5727549486185954, 0.06481958907187409)
    # degree_set = radius2degree(radius_set)
    # print(degree_set)