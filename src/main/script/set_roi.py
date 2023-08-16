import cv2
import numpy as np
from vein_recognize import get_vein
from helpers import colorize
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def get_roi(msg):
    global ir
    bridge = CvBridge()
    ir = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')



def main():
    global ir
    ir = None
    selected = False
    
    while True:
        if ir is None:
            continue
        frame = ir
        # frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))

        if not selected:
            select = cv2.selectROI('frame', colorize(frame, (None, 5000), cv2.COLORMAP_BONE), False)
        if select != (0, 0, 0, 0):
            selected = True
        else:
            continue

        with open('roi.txt', 'w') as f:
            f.write(str(select))
        print('ROI setted.')

        # cv2.imwrite('ir.jpg', colorize(frame, (None, 5000), cv2.COLORMAP_BONE))

        cv2.destroyAllWindows()
        break


if __name__ == '__main__':
    rospy.init_node('set_roi')
    rospy.Subscriber('/camera/ir', Image, get_roi)
    main()