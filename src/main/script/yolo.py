#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import math 
from contest_msgs.msg import *

# model
model = YOLO("/yolo/ult/ss-best-24.pt")
# object classes
classNames = ['base', 'cotton-swab', 'tube']

bridge = CvBridge()

rospy.init_node('yolo')
rospy.loginfo('yolo node start')
pub = rospy.Publisher('/yolo', yolo, queue_size=10)

def callback(data):
    img = bridge.imgmsg_to_cv2(data, "passthrough")
    results = model(img, stream=True, half=True)
    # coordinates
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
            # put box in cam
            # cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)
            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])
            msg = yolo()
            info = yolo_info()
            info.confidence = confidence
            info.class_name = classNames[cls]
            info.x1 = x1
            info.y1 = y1
            info.x2 = x2
            info.y2 = y2
            msg.info = info
            pub.publish(msg)

sub = rospy.Subscriber("/camera/color", Image, callback)
rospy.spin()
