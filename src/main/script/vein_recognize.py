import cv2
import numpy as np
from helpers import colorize

def get_vein(img, select):
    original = img.copy()
    # cv2.imshow('original', original)
    b, g, r = cv2.split(img)
    # use clahe to enhance the contrast
    clahe = cv2.createCLAHE(clipLimit=25, tileGridSize=(8,8))
    b = clahe.apply(img[:,:,0])
    g = clahe.apply(img[:,:,1])
    r = clahe.apply(img[:,:,2])
    strong = cv2.merge([b,g,r])

    strong = cv2.cvtColor(strong, cv2.COLOR_BGR2GRAY)

    # gaussian blur
    strong = cv2.GaussianBlur(strong, (3,3), 0)

    # use roi to extract the strong part at the center
    roi = strong

    # cv2.imshow('strong', strong)

    # ret, thresh = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    thresh = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 5)

    for i in range(10):
        thresh = cv2.dilate(thresh, None, iterations=1)
        thresh = cv2.erode(thresh, None, iterations=1)
    thresh = cv2.Canny(thresh, 180, 200)

    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow('thresh', thresh)

    # find contours most like a line
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)


    # roi = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)

    def contourArea(cnt):
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        area = cv2.contourArea(box)
        return area

    # find the contour with the largest area
    contours = sorted(contours, key=contourArea, reverse=True)

    # 排除面積過大的輪廓
    # area為截取的面積
    area = select[2] * select[3] * 0.8
    contours = [i for i in contours if contourArea(i) < area]

    cnt = contours[0]


    # # draw the contour
    # cv2.drawContours(img, cnt, -1, (0,255,0), 2)

    # # draw the bounding box
    # for i in cnt:
    #     rect = cv2.minAreaRect(i)
    #     box = cv2.boxPoints(rect)
    #     box = np.int0(box)
    #     cv2.drawContours(img, [box], -1, (0,0,255), 2)

    center = cv2.minAreaRect(cnt)[0]

    cv2.circle(strong, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

    return strong, center



if __name__ == '__main__':
    import pyk4a
    from pyk4a import Config, PyK4A
    k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_1080P,
                        depth_mode=pyk4a.DepthMode.WFOV_UNBINNED,
                        camera_fps=pyk4a.FPS.FPS_15))
    k4a.start()
    with open('roi.txt', 'r') as f:
        select = eval(f.read())
    while True:
        ir = k4a.get_capture().transformed_ir

        roi_points = [(select[0], select[1]), (select[0] + select[2], select[1]), (select[0] + select[2], select[1] + select[3]), (select[0], select[1] + select[3])]
        
        cv2.imwrite('ir.jpg', colorize(ir, (None, 5000), cv2.COLORMAP_BONE))
        _img = cv2.imread('ir.jpg', 0)
        roi = np.zeros(_img.shape, dtype=np.uint8)
        cv2.fillPoly(roi, [np.array(roi_points)], (255, 255, 255))
        _img = cv2.bitwise_and(_img, roi)
        arm = _img

        # roi = cv2.rotate(roi, cv2.ROTATE_180)

        arm = cv2.cvtColor(arm, cv2.COLOR_GRAY2BGR)
        arm, vein_target = get_vein(arm)

        cv2.imshow('arm', arm)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f'chosen: {vein_target}')
            break
        elif key == ord('r'):
            continue