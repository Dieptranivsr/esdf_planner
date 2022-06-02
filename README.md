# esdf_planner
separate esdf package from [fast_planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

<img src="https://user-images.githubusercontent.com/69444682/160658182-89b4e271-4fe5-4c73-b2d9-41fdee4f34c7.jpg" width="500">

## Initialize Px4 (Get GPS data)
```shell
  $ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
```

## Initialize Depth Camera D435
```shell
  $ roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=15
  $ roslaunch realsense2_camera rs_camera.launch enable_color:=false depth_width:=640 depth_height:=480 depth_fps:=15
```

## Roslaunch mains_node
```shell
  $ roslaunch esdf_planner exp_esdf_planner.launch
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
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)|(.*)global_position(.*)|(.*)imu(.*)|(.*)geometric_controller(.*)|(.*)reference(.*)"
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)|(.*)global_position(.*)|(.*)imu(.*)|(.*)geometric_controller(.*)|(.*)reference(.*)|(.*)setpoint_raw(.*)|(.*)rgb_camera(.*)|(.*)stereo_module(.*)"
```

## Simulation
```shell
$ roslaunch esdf_planner sim.launch
$ roslaunch esdf_planner hitl_esdf_planner.launch
```
```shell
$ roslaunch esdf_planner px4_esdf_planner.launch
```
```shell
$ rostopic pub --once /command/trajectory trajectory_msgs/MultiDOFJointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- transforms:
  - translation:
      x: -2.0
      y: 2.0
      z: 1.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  velocities:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  accelerations:
  - linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  time_from_start:
    secs: 0
    nsecs: 0" 
publishing and latching message for 3.0 seconds
```

## Experiment
```shell
$ roslaunch px4_fast_planner mavros.launch
```
```shell
$ roslaunch px4_fast_planner geometric_controller.launch
```
```shell
# SITL
$ roslaunch esdf_planner hitl_esdf_planner.launch
```
```shell
$ export GAZEBO_MODEL_PATH=`pwd`/models
```
