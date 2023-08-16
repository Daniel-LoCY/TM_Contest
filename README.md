# 達明比賽2023

## 使用說明

1. 先用arm_to_aruco.py找出aruco中心點在手臂座標系的座標
2. 再用aruco.py找出aruco中心點在相機座標系的座標
3. 最後用calc_coord.py的camera2arm()將相機座標系的座標轉換成在手臂座標系的座標

## Docker

### [Image](???)

docker run --rm -it --gpus all --net=host -v {local_path}:{container_path} -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix {image_name}