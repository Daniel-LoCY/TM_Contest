# 公用程式

import math
import numpy as np
import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tm_msgs.srv import *
from tm_msgs.msg import *

def euler_to_orientation(roll, pitch, yaw):
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    # Calculate quaternion components
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    # Normalize quaternion
    length = math.sqrt(w**2 + x**2 + y**2 + z**2)
    w /= length
    x /= length
    y /= length
    z /= length
    
    return np.array([x, y, z, w])

def camera2arm(x, y):

    # 好像用不到
    # with open("camera_origin_in_marker.txt", "r") as f:
    #     # print(f.read())
    #     camera_origin_in_marker = eval(f.read())
    # print("相機在標記中的位置：", camera_origin_in_marker)

    with open("point_at_arm.txt", "r") as f:
        aruco_center_in_arm = eval(f.read())
        # mm to m
        aruco_center_in_arm = np.array(aruco_center_in_arm, dtype=np.float32) / 1000

    # print("aruco中心點在手臂座標系中的位置：", aruco_center_in_arm)

    target = np.array([x, y], dtype=np.float32) # 目標點在相機的pixel座標
    
    with open("center.txt", "r") as f:
        center = eval(f.read())
        center_x = center[0]
        center_y = center[1]

    # 計算目標點與aruco中心點的x, y距離
    x_distance = target[0] - center_x
    y_distance = target[1] - center_y

    # print("目標點與aruco中心點的x, y距離：", x_distance, y_distance)

    with open("pixel_per_meter.txt", "r") as f:
        pixel_per_meter = eval(f.read())

    # 將pixel距離轉換成真實距離
    x_distance = x_distance / pixel_per_meter
    y_distance = y_distance / pixel_per_meter
    # print("目標點與aruco中心點的真實距離：", x_distance, y_distance)

    # 計算目標點在手臂座標系中的位置
    # x加負號是因為相機的x軸與手臂座標系的x軸相反
    target_point_in_arm = np.array([-x_distance, y_distance, 0], dtype=np.float32) + aruco_center_in_arm

    # m to mm
    target_point_in_arm = target_point_in_arm * 1000

    # print("目標點在手臂座標系中的位置：", target_point_in_arm, "\n")

    return target_point_in_arm

def pixel2mm(x, y):

    with open("pixel_per_meter.txt", "r") as f:
        pixel_per_meter = eval(f.read())

    # 將pixel距離轉換成真實距離
    x_distance = x / pixel_per_meter * 1000
    y_distance = y / pixel_per_meter * 1000

    return x_distance, y_distance

def depthCamera2arm(x, y):

    with open("point_at_arm.txt", "r") as f:
        aruco_center_in_arm = eval(f.read())
        # mm to m
        aruco_center_in_arm = np.array(aruco_center_in_arm, dtype=np.float32) / 1000

    # print("aruco中心點在手臂座標系中的位置：", aruco_center_in_arm)

    target = np.array([x, y], dtype=np.float32) # 目標點在相機的pixel座標
    
    with open("depth_center.txt", "r") as f:
        center = eval(f.read())
        center_x = center[0]
        center_y = center[1]

    # 計算目標點與aruco中心點的x, y距離
    x_distance = target[0] - center_x
    y_distance = target[1] - center_y

    # print("目標點與aruco中心點的x, y距離：", x_distance, y_distance)

    with open("depth_pixel_per_meter.txt", "r") as f:
        pixel_per_meter = eval(f.read())

    # 將pixel距離轉換成真實距離
    x_distance = x_distance / pixel_per_meter
    y_distance = y_distance / pixel_per_meter
    # print("目標點與aruco中心點的真實距離：", x_distance, y_distance)

    # 計算目標點在手臂座標系中的位置
    # x加負號是因為相機的x軸與手臂座標系的x軸相反
    target_point_in_arm = np.array([-x_distance, y_distance, 0], dtype=np.float32) + aruco_center_in_arm

    # m to mm
    target_point_in_arm = target_point_in_arm * 1000

    # print("目標點在手臂座標系中的位置：", target_point_in_arm, "\n")

    return target_point_in_arm

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

# 持續判斷手臂是否到達目標姿態
def check_arm_reach_target(arm_pose, target_pose, threshold=0.1):
    arm_pose = list(arm_pose)
    target_pose = list(target_pose)
    for i in range(3, 6):
        arm_pose[i] = arm_pose[i] + 360 if arm_pose[i] % 360 < 0 else arm_pose[i]
        target_pose[i] = target_pose[i] + 360 if target_pose[i] % 360 < 0 else target_pose[i]
    # print("\n","arm_pose: ", arm_pose)
    # print("target_pose: ", target_pose)
    # print(abs(arm_pose[0] - target_pose[0]), abs(arm_pose[1] - target_pose[1]), abs(arm_pose[2] - target_pose[2]), abs(arm_pose[3] - target_pose[3]), abs(arm_pose[4] - target_pose[4]), abs(arm_pose[5] - target_pose[5]))
    if abs(arm_pose[0] - target_pose[0]) < threshold and abs(arm_pose[1] - target_pose[1]) < threshold and abs(arm_pose[2] - target_pose[2]) < threshold and abs(arm_pose[3] - target_pose[3]) < threshold and abs(arm_pose[4] - target_pose[4]) < threshold and abs(arm_pose[5] - target_pose[5]) < threshold:
        return True
    else:
        return False
    
def euler_to_orientation(roll, pitch, yaw):
    '''
    Convert euler angles to quaternion \n
    euler: (roll, pitch, yaw) in degrees \n
    quaternion: (x, y, z, w)
    '''
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    # Calculate quaternion components
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    # Normalize quaternion
    length = math.sqrt(w**2 + x**2 + y**2 + z**2)
    w /= length
    x /= length
    y /= length
    z /= length
    
    return np.array([x, y, z, w])

def orientation_to_euler(x, y, z, w):
    '''
    Convert quaternion to euler angles \n
    euler: (roll, pitch, yaw) in degrees \n
    quaternion: (x, y, z, w)
    '''
    # Convert quaternion components to euler angles in radians
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    # Convert to degrees
    roll_x = math.degrees(roll_x)
    pitch_y = math.degrees(pitch_y)
    yaw_z = math.degrees(yaw_z)
    
    return np.array([roll_x, pitch_y, yaw_z])

def pub_tf(x, y, z, rx, ry, rz, header_frame_id="arm", child_frame_id="tool"):
    br = TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    quat = euler_to_orientation(rx, ry, rz)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    br.sendTransform(t)

def pub_tf_static(x, y, z, rx, ry, rz, header_frame_id="tool", child_frame_id="tool_target"):
    br = StaticTransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    quat = euler_to_orientation(rx, ry, rz)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    br.sendTransform(t)
    # rospy.loginfo(f"pub_tf_static: {t}")

def pub_tf_static_orientation(x, y, z, rx, ry, rz, rw, header_frame_id, child_frame_id):
    br = StaticTransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    t.transform.rotation.x = rx
    t.transform.rotation.y = ry
    t.transform.rotation.z = rz
    t.transform.rotation.w = rw
    br.sendTransform(t)
    # rospy.loginfo(f"pub_tf_static: {t}")


def get_tf(header_frame, child_frame): # 取得tf,  return transform
    tfBuffer = Buffer()
    listener = TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform(header_frame, child_frame, rospy.Time())
            transform = t.transform
            print(transform)
            # translation from m to mm
            transform.translation.x = transform.translation.x * 1000
            transform.translation.y = transform.translation.y * 1000
            transform.translation.z = transform.translation.z * 1000
            # rotation from quaternion to euler
            euler = orientation_to_euler(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
            # radius to degree
            transform.rotation.x = euler[0]
            transform.rotation.y = euler[1]
            transform.rotation.z = euler[2]
            return transform
        except Exception as e:
            # print(e)
            pass

def get_target_z(x=1, y=1, depth=3.5, depth_floor=4):

    print("x, y, depth, depth_floor: ", x, y, depth, depth_floor)

    if depth**2 < x**2 + y**2:
        print("No solution")
        return None

    z = depth_floor - math.sqrt(abs((depth**2)-(x**2)-(y**2)))
    
    return z