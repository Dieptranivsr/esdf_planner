# esdf_planner
separate esdf package from [fast_planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

![1857017bf72939776038](https://user-images.githubusercontent.com/69444682/160658182-89b4e271-4fe5-4c73-b2d9-41fdee4f34c7.jpg)

## Initialize Px4 (Get GPS data)
```shell
  $ roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

## Initialize Depth Camera D435
```shell
  $ roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=15
```

## Roslaunch mains_node
```shell
  $ roslaunch esdf_planner exp_esdf_planner.launch
  $ roslaunch esdf_planner hitl_esdf_planner.launch
```
```shell
  $ rosrun tf view_frames && evince frames.pdf
```
```shell
  $ rosrun tf tf_echo map base_link
```

## Record bag files. Furthermore, Reduce bag file's capacity
```shell
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)"
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)|(.*)global_position(.*)|(.*)imu(.*)"
```
