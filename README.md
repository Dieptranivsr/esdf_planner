# esdf_planner
separate esdf package in fast_planner
```shell
  $ roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```
```shell
  $ roslaunch esdf_planner exp_esdf_planner.launch
```
```shell
  $ roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=15
```
```shell
  $ rosrun tf view_frames && evince frames.pdf
```
```shell
  $ rosrun tf tf_echo map base_link
```
