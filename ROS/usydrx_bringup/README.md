# usydrx_bringup

This ros package contails all the launch files needed to bring up the system. It also contains some misc scripts / nodes.

## Nodes
### heartbeat.py
Used for the heartbeat task for RobotX.
It will connect to the relevant server "robot.server"
It will subscribe the the relevant topics and output a heartbeat at once second relevant the the current task it is operating.

### judge_display.py

The judge display node is used for the light display task it will present a gui that provide a visual feedback of the light sequence detected. 

### test_server.py
This is the heartbeat test server. It can be used to validate the heartbeat.py task.

## Launch Files
### base_bringup.launch
Bring this up when connected to the wamv over 900 Mhz and running a task. It will launch the "basic rviz view" and heartbeat.

### basic_view_rviz.launch
Brings up a vew low bandwith version of the rviz view. It bring up rviz with only low bandwidth topics. Topics such as the camera feeds and lidar is omitted because it would not travel effectively only the 900 Mhz. It has topics such as the odometry and obstacle topics.

### bringup_sim_control_core.launch
Brings up the simulation and the control code.

### bringup_sim_with_object_classifier_and_path_following_control.launch
Brings up the simulation with control, the object classifier and path following


### 

