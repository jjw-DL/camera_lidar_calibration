# 相机与激光雷达联合标定

![img](https://github.com/jjw-DL/camera_lidar_calibration/blob/master/cam_lidar_calibration/images/calibration.gif) 

## 参考博客
https://adamshan.blog.csdn.net/article/details/105736726

## 编译
```bash
mkdir -r cam_lidar_calibration_ws/src
cd  cam_lidar_calibration_ws/src
git clone git@github.com:jjw-DL/camera_lidar_calibration.git
cd ..
catkin_make

```
##  运行
```bash
roslaunch velodyne_pointcloud VLP-32C_points.launch
roslaunch cam_lidar_calibration cam_lidar_calibration.launch 
rosbag play camera_lidar.bag
加载rviz：
     rviz/cam_lidar_calibration.rviz
完成标定后，将lidar投射到camera上：
    rosbag play camera_lidar.bag
    roslaunch cam_lidar_calibration  project_img.launch 
```
## 数据集
链接：https://pan.baidu.com/s/1hmxkY0scBc-qEC4A208PbA 
提取码：fj19 
