import cv2
import numpy as np
import pyk4a
from pyk4a import Config, PyK4A
from helpers import colorize

# 定義ArUco字典
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)

# 建立ArUco檢測器
aruco_params = cv2.aruco.DetectorParameters_create()

# 設定已知的ArUco標記大小（以米為單位）
marker_size = 0.08


k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_1080P,
                     depth_mode=pyk4a.DepthMode.NFOV_2X2BINNED,
                     disable_streaming_indicator=True))
k4a.start()

camera_matrix = k4a.calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.COLOR)

dist_coeffs = k4a.calibration.get_distortion_coefficients(pyk4a.calibration.CalibrationType.COLOR)

while True:
    cap = k4a.get_capture()
    frame = cap.color
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    if frame is None:
        continue

    # 檢測ArUco標記
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # 繪製檢測到的標記
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # 計算相機姿態
        _rvecs, _tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            id = ids[i][0]
            tvecs = _tvecs[i][0]
            rvecs = _rvecs[i][0]
            
            # print(f"ArUco ID: {id}, 位置: {tvecs}, 旋轉向量: {rvecs}")

            cv2.circle(frame, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 5, (0, 0, 255), -1)
            cv2.circle(frame, (int(corners[0][0][1][0]), int(corners[0][0][1][1])), 5, (0, 255, 0), -1)
            cv2.circle(frame, (int(corners[0][0][2][0]), int(corners[0][0][2][1])), 5, (255, 0, 0), -1)
            cv2.circle(frame, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), 5, (255, 255, 0), -1)

            # 計算長寬
            width = int(np.sqrt((corners[0][0][0][0] - corners[0][0][1][0]) ** 2 + (corners[0][0][0][1] - corners[0][0][1][1]) ** 2))
            height = int(np.sqrt((corners[0][0][0][0] - corners[0][0][3][0]) ** 2 + (corners[0][0][0][1] - corners[0][0][3][1]) ** 2))
            # print("長寬：", width, height)

            # 計算真實距離與pixel的比例
            pixel_per_meter = width / marker_size
            # print("真實距離與pixel的比例：", pixel_per_meter)

            with open('pixel_per_meter.txt', 'w') as f:
                f.write(str(pixel_per_meter))

            # 計算中心點
            center_x = int((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4)
            center_y = int((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4)

            with open('center.txt', 'w') as f:
                f.write(str((center_x, center_y)))

            # print("中心點：", center_x, center_y)

            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)

            rvecs_matrix, _ = cv2.Rodrigues(rvecs)
            
            camera_origin_in_marker = -np.dot(np.linalg.inv(rvecs_matrix), tvecs)

            rmat, _ = cv2.Rodrigues(rvecs)
            
            print("彩色相機在標記中的位置：", camera_origin_in_marker)

            # with open('camera_origin_in_marker.txt', 'w') as f:
            #     f.write(str(list(camera_origin_in_marker))

    
    # 這裡是深度相機的部分
    ir = cap.transformed_ir


    camera_matrix = k4a.calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.DEPTH)

    dist_coeffs = k4a.calibration.get_distortion_coefficients(pyk4a.calibration.CalibrationType.DEPTH)


    # 不知道為什麼這樣才可以
    cv2.imwrite('ir.png', colorize(ir, (None, 5000), cv2.COLORMAP_BONE))
    ir = cv2.imread('ir.png')

    if ir is None:
        continue

    # 檢測ArUco標記
    corners, ids, rejected = cv2.aruco.detectMarkers(ir, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # 繪製檢測到的標記
        ir = cv2.aruco.drawDetectedMarkers(ir, corners, ids)

        # 計算相機姿態
        _rvecs, _tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            id = ids[i][0]
            tvecs = _tvecs[i][0]
            rvecs = _rvecs[i][0]
            
            # print(f"ArUco ID: {id}, 位置: {tvecs}, 旋轉向量: {rvecs}")

            cv2.circle(ir, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 5, (0, 0, 255), -1)
            cv2.circle(ir, (int(corners[0][0][1][0]), int(corners[0][0][1][1])), 5, (0, 255, 0), -1)
            cv2.circle(ir, (int(corners[0][0][2][0]), int(corners[0][0][2][1])), 5, (255, 0, 0), -1)
            cv2.circle(ir, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), 5, (255, 255, 0), -1)

            # 計算長寬
            width = int(np.sqrt((corners[0][0][0][0] - corners[0][0][1][0]) ** 2 + (corners[0][0][0][1] - corners[0][0][1][1]) ** 2))
            height = int(np.sqrt((corners[0][0][0][0] - corners[0][0][3][0]) ** 2 + (corners[0][0][0][1] - corners[0][0][3][1]) ** 2))
            # print("長寬：", width, height)

            # 計算真實距離與pixel的比例
            pixel_per_meter = width / marker_size
            # print("真實距離與pixel的比例：", pixel_per_meter)

            with open('depth_pixel_per_meter.txt', 'w') as f:
                f.write(str(pixel_per_meter))

            # 計算中心點
            center_x = int((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4)
            center_y = int((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4)

            with open('depth_center.txt', 'w') as f:
                f.write(str((center_x, center_y)))

            # print("中心點：", center_x, center_y)

            cv2.circle(ir, (center_x, center_y), 5, (0, 255, 255), -1)

            rvecs_matrix, _ = cv2.Rodrigues(rvecs)
            
            camera_origin_in_marker = -np.dot(np.linalg.inv(rvecs_matrix), tvecs)

            rmat, _ = cv2.Rodrigues(rvecs)
            
            print("深度相機在標記中的位置：", camera_origin_in_marker)

            # with open('depth_camera_origin_in_marker.txt', 'w') as f:
            #     f.write(str(list(camera_origin_in_marker)))

    cv2.imshow('color', frame)
    cv2.imshow('depth', ir)

    if cv2.waitKey(1) == ord('q'):
        break

k4a.stop()
cv2.destroyAllWindows()


