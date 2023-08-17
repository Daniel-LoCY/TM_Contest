#!/usr/bin/env python3

import rospy
from util import *
from tm_msgs.srv import *
from tm_msgs.msg import *
from contest_msgs.msg import *
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from helpers import colorize
from vein_recognize import get_vein
import threading


rospy.init_node('main')

bridge = CvBridge()

def feedback_callback(msg: FeedbackState): # 手臂姿態
    global ax, ay, az, arx, ary, arz, arm_pose, feedback_callback_flag
    ax = msg.tool_pose[0]*1000
    ay = msg.tool_pose[1]*1000
    az = msg.tool_pose[2]*1000
    arx = msg.tool_pose[3]*180/3.1415926
    ary = msg.tool_pose[4]*180/3.1415926
    arz = msg.tool_pose[5]*180/3.1415926
    arm_pose = (ax, ay, az, arx, ary, arz)
    # rospy.loginfo(f"arm_pose: {arm_pose}")
    tool_pose = (msg.tool_pose[0], msg.tool_pose[1], msg.tool_pose[2], msg.tool_pose[3], msg.tool_pose[4], msg.tool_pose[5])
    # rospy.loginfo(f"tool_pose: {tool_pose}")
    
    pub_tf_static(tool_pose[0], tool_pose[1], tool_pose[2], tool_pose[3], tool_pose[4], tool_pose[5], "arm", "tool")


def pub_tf_always():
    while True:
        rospy.loginfo("pub_tf_always")
        rospy.sleep(0.1)

def check(target_pose, threshold=0.1): # 檢查手臂是否到達目標點
    global arm_pose
    while True:
        rospy.loginfo("Waiting for arm to reach target pose...")
        if check_arm_reach_target(arm_pose=arm_pose, target_pose=target_pose, threshold=threshold):
            rospy.loginfo("Reach target pose")
            break
        # rospy.sleep(0.1)
        # break

def yolo_callback(msg: yolo): # yolo辨識結果
    global x, y, class_name, confidence, color, x1, y1, x2, y2, status
    rospy.loginfo(msg)
    print()
    info = msg.info
    _class_name = info.class_name
    _confidence = info.confidence
    if _class_name in status and _confidence > 0.5:
        rospy.loginfo(f"detected: {_class_name}, {_confidence}")
        class_name = info.class_name
        confidence = info.confidence
        x1 = info.x1
        y1 = info.y1
        x2 = info.x2
        y2 = info.y2
        x = (x1+x2)/2
        y = (y1+y2)/2
        # rospy.loginfo(f"yolo: {x}, {y}")

def readyPose():
    cmd = f"PTP(\"CPP\",200,220, 60,180,0,180,200,50,0,false,0,2,4)"
    resp = tm_send_script_client(cmd)
    tm_send_gripper_client(False)
    # check_arm_reach_target(arm_pose=arm_pose, target_pose=(391,300, -180,180,0,180))

def camera_color_callback(msg: Image):
    global color
    color = bridge.imgmsg_to_cv2(msg, "passthrough")

def camera_depth_callback(msg: Image):
    global depth
    depth = bridge.imgmsg_to_cv2(msg, "passthrough")

def camera_ir_callback(msg: Image):
    global ir
    ir = bridge.imgmsg_to_cv2(msg, "passthrough")

sub_feedback = rospy.Subscriber('feedback_states', FeedbackState, feedback_callback)
sub_camera_color = rospy.Subscriber('/camera/color', Image, camera_color_callback)
sub_camera_depth = rospy.Subscriber('/camera/depth', Image, camera_depth_callback)
sub_camera_ir = rospy.Subscriber('/camera/ir', Image, camera_ir_callback)
sub_yolo = rospy.Subscriber('/yolo', yolo, yolo_callback)

x, y = 0, 0

error_x = 0
error_y = 0

# rospy.sleep(1)

delay = 5
z_low_floor = 37 # z軸夾取最低點 mm
speed = 200
m = 100
status = ["cotton-swab"]

# t = threading.Thread(target=pub_tf_always)
# t.daemon = True
# t.start()
# rospy.sleep(5)
# exit()

# x1, y1, x2, y2, color = 0, 0, 0, 0, None
# def bbx():
#     global x1, y1, x2, y2, color
#     cv2.rectangle(color, (x1, y1), (x2, y2), (255, 0, 255), 3)
#     cv2.imshow('color', color)
#     cv2.waitKey(1)

# t = threading.Thread(target=bbx)
# t.daemon = True

'''
# while x == 0 and y == 0:
#     rospy.loginfo("Waiting for yolo...")
#     rospy.sleep(0.1)

# # 判斷x,y是否在一定範圍內維持3秒
# import time

# def check_range(x, y, target_range_x, target_range_y):
#     start_time = time.time()
    
#     while time.time() - start_time < 3:
#         if target_range_x[0] < x < target_range_x[1] and target_range_y[0] < y < target_range_y[1]:
#             continue
#         else:
#             return False
#     return True

# target_range_x = (x-10, x+10)
# target_range_y = (y-10, y+10)

# if check_range(x, y, target_range_x, target_range_y):
#     rospy.loginfo("x和y在範圍內維持了3秒。")
# else:
#     rospy.loginfo("x和y不在範圍內維持了3秒。")
'''

# Start

# 1. 準備動作

readyPose()
# exit()
rospy.sleep(1)

# t.start()



# 2. 辨識血管

rospy.loginfo("Press s to start")

select = None
with open('roi.txt', 'r') as f:
    select = eval(f.read())
    roi_points = [(select[0], select[1]), (select[0] + select[2], select[1]), (select[0] + select[2], select[1] + select[3]), (select[0], select[1] + select[3])]

while True:
    
    cv2.imwrite('ir.jpg', colorize(ir, (None, 5000), cv2.COLORMAP_BONE))
    _img = cv2.imread('ir.jpg', 0)
    roi = np.zeros(_img.shape, dtype=np.uint8)
    cv2.fillPoly(roi, [np.array(roi_points)], (255, 255, 255))
    _img = cv2.bitwise_and(_img, roi)

    arm = cv2.cvtColor(_img, cv2.COLOR_GRAY2BGR)
    arm, vein_target = get_vein(arm, select)
    # print(f'vein_target: {vein_target}')

    cv2.imshow('arm', arm)
    key = cv2.waitKey(0)
    if key == ord('q'):
        exit()
    elif key == ord('s'):
        depth_val = depth[int(vein_target[1]), int(vein_target[0])]
        print(f'chosen: {vein_target}, depth: {depth_val}')
        break
    elif key == ord('r'):
        continue

# depth_val = depth[y, x]

target = depthCamera2arm(vein_target[0], vein_target[1])
xx, yy = pixel2mm(depth.shape[1]//2-1-vein_target[0], depth.shape[0]//2-1-vein_target[1])
vein_height = get_target_z(xx, yy, depth_val, depth[depth.shape[0]//2-1, depth.shape[1]//2-1])+z_low_floor

rospy.loginfo(f"vein_height: {vein_height}")

if vein_height < 45:
    exit()


vein_target = (target[0], target[1], vein_height)
# exit()
target_pose = (vein_target[0], vein_target[1], vein_target[2], 180, 0, 180) # z=depth_value
rospy.loginfo(f"target_pose: {target_pose}")
cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
resp = tm_send_script_client(cmd)
rospy.sleep(10)
# tm_send_gripper_client(True)
# exit()
# rospy.sleep(10)
tm_send_gripper_client(False)
readyPose()


# 3. 消毒
rospy.sleep(3)
rospy.loginfo(f"Status: {status}")

while True:
    # break
    try:
        if class_name == "cotton-swab":
            rospy.loginfo("detect cotton-swab")
            rospy.loginfo(f"x, y: {x}, {y}")
            target = camera2arm(x, y)
            origin_target = target
            rospy.loginfo(f"target: {target}")
            target_pose = (target[0], target[1], z_low_floor+50, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(8)
            target_pose = (target[0], target[1]-20, z_low_floor, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(3)
            tm_send_gripper_client(True)
            rospy.sleep(5)
            target_pose = (vein_target[0], vein_target[1], vein_target[2], 180, 0, 180) # z=depth_value
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(5)

            cmd = f"Move_PTP(\"CPP\",0,-50,0,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)
            cmd = f"Move_PTP(\"CPP\",0,0,0,-12,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)
            cmd = f"Move_PTP(\"CPP\",0,0,-8,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)

            for i in range(30):
                now_pose = arm_pose # 紀錄現在手臂的位置
                rospy.sleep(0.1)
            transform = get_tf("base", "tool_target_cotton")
            
            for i in range(3):
                target_pose = (transform.translation.x, transform.translation.y, transform.translation.z, 
                            transform.rotation.x, transform.rotation.y, transform.rotation.z)
                cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
                resp = tm_send_script_client(cmd)
                rospy.sleep(1)

                target_pose = now_pose
                cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
                resp = tm_send_script_client(cmd)
                rospy.sleep(1)

            target_pose = (origin_target[0], origin_target[1], z_low_floor, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            tm_send_gripper_client(False)
            
            # for i in range(1):
            #     cmd = f"Move_PTP(\"CPP\",0,0,0,0,0,20,{speed},{m},0,false)"
            #     resp = tm_send_script_client(cmd)
            #     rospy.sleep(3)
            #     cmd = f"Move_PTP(\"CPP\",0,0,0,0,0,-40,{speed},{m},0,false)"
            #     resp = tm_send_script_client(cmd)
            #     rospy.sleep(3)
            #     cmd = f"Move_PTP(\"CPP\",0,0,0,0,0,20,{speed},{m},0,false)"
            #     resp = tm_send_script_client(cmd)
            #     rospy.sleep(3)

            # readyPose()
            rospy.sleep(3)

            break
        else:
            rospy.loginfo("No cotton-swab")
            rospy.sleep(0.1)
            continue
    except Exception as e: 
        rospy.loginfo(f"No detection {e}")
        rospy.sleep(0.1)

readyPose()
        

# exit()
# 4. 抽血
rospy.sleep(3)
status = ["syringe", "syringe-full"]
rospy.loginfo(f"Status: {status}")

while True:
    # break
    try:
        if class_name in status:
            rospy.loginfo(f"detect {status}")
            target = camera2arm(x, y)
            origin_target = target
            target_pose = (target[0], target[1]-20, z_low_floor+50, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose)
            rospy.sleep(8)
            target_pose = (target[0], target[1]-20, z_low_floor, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose)
            rospy.sleep(3)
            tm_send_gripper_client(True)
            # break
            rospy.sleep(3)
            cmd = f"Move_PTP(\"CPP\",0,0,40,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)


            target_pose = (vein_target[0], vein_target[1]-125, vein_target[2], 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose)
            rospy.sleep(3)

            # rospy.loginfo("check")

            cmd = f"Move_PTP(\"CPP\",0,0,10,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)

            cmd = f"Move_PTP(\"CPP\",0,0,0,-16.5,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)

            cmd = f"Move_PTP(\"CPP\",0,0, 0,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)

            # exit()

            for i in range(80):
                now_pose = arm_pose # 紀錄現在手臂的位置
                rospy.sleep(0.1)
            transform = get_tf("base", "tool_target")
            rospy.loginfo(f"transform: {transform}")
            target_pose = (transform.translation.x, transform.translation.y, transform.translation.z, 
                        transform.rotation.x, transform.rotation.y, transform.rotation.z)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose)
            rospy.sleep(10)
            # rospy.sleep(30) # 等待抽血

            while class_name != "syringe-full":
                rospy.loginfo("Waiting for syringe-full...")
                rospy.sleep(0.01)

            target_pose = now_pose
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose)
            rospy.sleep(3)
            
            target_pose = (origin_target[0], origin_target[1], z_low_floor, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            tm_send_gripper_client(False)

            readyPose()

            break
        else:
            rospy.loginfo("No base")
            rospy.sleep(0.1)
    except Exception as e: 
        rospy.loginfo(f"No detection {e}")
        rospy.sleep(0.1)

# exit()

# 5. ???
# 3. 消毒
rospy.sleep(3)
status = ["cotton-ball"]
rospy.loginfo(f"Status: {status}")

while True:
    # break
    try:
        if class_name in status:
            rospy.loginfo(f"detect {status}")
            # rospy.loginfo(x, y)
            target = camera2arm(x, y)
            rospy.loginfo(f"target: {target}")
            target_pose = (target[0], target[1], z_low_floor+50, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(8)
            target_pose = (target[0], target[1], z_low_floor+10, 180, 0, 180)
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(3)
            tm_send_gripper_client(True)
            rospy.sleep(5)
            rospy.loginfo(f"vein_target: {vein_target}")
            target = camera2arm(vein_target[0], vein_target[1])
            rospy.loginfo(f"target: {target}")
            target_pose = (target[0]-error_x, target[1]-error_y, z_low_floor+150, 180, 0, 180) # z=depth_value
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(5)
            target_pose = (target[0]+8, target[1], z_low_floor+100, 180, 0, 180) # z=depth_value
            cmd = f"PTP(\"CPP\",{target_pose}, {speed}, {m}, 0, false, 0, 2, 4)"
            resp = tm_send_script_client(cmd)
            # check(target_pose, threshold=0.5)
            rospy.sleep(5)

            cmd = f"Move_PTP(\"CPP\",0,35,0,0,0,0,{speed},{m},0,false)"
            resp = tm_send_script_client(cmd)
            rospy.sleep(3)

            readyPose()
            rospy.sleep(3)

            break
        else:
            rospy.loginfo(f"No {status}")
            rospy.sleep(0.1)
            continue
    except Exception as e: 
        rospy.loginfo(f"No detection {e}")
        rospy.sleep(0.1)