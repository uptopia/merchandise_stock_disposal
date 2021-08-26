# mrk_relative_pose

## STEP 1
```
roscore
```


## STEP2: ROS Server <terminal 1> 
```
cd ~/merchandise_stock_disposal
. devel/setup.bash
chmod +x src/mrk_relative_pose/src/MoveRelativePose_Mobile_server.py

rosrun mrk_relative_pose MoveRelativePose_Mobile_server.py
```

## STEP3: ROS Client <terminal 2> 
## [Test1] Client (Arm) send request destination to Server (Mobile) 
```
cd ~/merchandise_stock_disposal
. devel/setup.bash
chmod +x src/mrk_relative_pose/src/MoveRelativePose_Mobile_client.py

rosrun mrk_relative_pose MoveRelativePose_Mobile_client.py 1.23 1.1 6.7
```