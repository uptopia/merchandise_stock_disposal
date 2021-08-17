# merchandise_stock_disposal


OPEN 1 Realsense
```
<terminal 1> Open Realsense D435i
cd ~/realsense_ros
. devel/setup.bash
roslaunch realsense2_camera rs_rgbd.launch
```

or 

OPEN 2 Realsense
```
cd ~/merchandise_stock_disposal
. devel/setup.bash
roslaunch aruco_detection open_dual_realsense.launch
```

EXECUTE Disposal and Stock Task
```
<terminal 2> Open TIMDA Dual Arm Gazebo
cd ~/merchandise_stock_disposal
. devel/setup.bash
roslaunch manipulator_h_manager dual_arm.launch en_sim:=true

<terminal 3> Detect ArUco Markers
cd ~/merchandise_stock_disposal
. devel/setup.bash
rosrun aruco_detection MarkerPosture.py

<terminal 4> Disposal and Stock Task
cd ~/merchandise_stock_disposal
. devel/setup.bash
rosrun strategy product_strategy.py True
```