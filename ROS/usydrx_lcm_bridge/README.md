# USYD_RX_LCM_BRIDGE

The usyd robotx lcm bridge package contains the code that will connect to the LCM software running on the WAM-V and port it to correspoding ROS Commands.

## nav_data_bridge.py

The nav data bridge takes in input from the gps, imu and relay from lcm and convert it to ROS commands.

**_NOTE:_**  The node was initially aimed to support nav messages (gps / imu) only but due to time limitiatons it was extended to support the relay messages from the WAM-V.  This should be move to another node.

## control_cmds_to_lcm.py

The control cmds to lcm node will convert the ROS motor control commands to corresponding lcm commands to control the motors. It will handling the scaling needed.

## pub_initial_pose.py

Pub inital pose will read from acfr nav and then publish the initial pose to odomoetry. This is so that there is a relevant inital pose determined that the navigation node can use. 