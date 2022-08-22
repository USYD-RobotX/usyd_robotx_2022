# robotx_2022
Repository to contain code for the 2022 Development of RobotX


The USYD RobotX Container contains the Code for the Development of the software for the usyd robotx 2022 team.

Quick Start
====================
```
mkdir -p ~/rx_ws/src

cd ~/rx_ws/src

git clone https://github.com/USYD-RobotX/usyd_robotx_2022.git

cd ~/rx_ws

catkin_make
```
The USYD RobotX Code is now ready to go on your system.

WAM_V Setup
===============================
How to setup and initailise the docker container on the wamv.

All the code will be run inside of a docker container. There is no need to install any packages (other than docker) to setup on the WAMV

SSH into the WAMV

`ssh auv@172.16.154.230`

Clone your repository into the system

`git clone https://github.com/USYD-RobotX/usyd_robotx_2022.git` 

Build the container

`docker build -t wamv_image -f wamv_Dockerfile .`


Run the container
`docker run --network=host -it --name wamv_container wamv_image:latest`


You should now be able to now run commands in the container. 
Ros should be avaliable to use.
Test using the command `roscore`

If you have started lcm on the wamv (run `cd ~ && start_lcm.sh`) you should be able to see the packets by running `pyspy`

Using a DEVCONTAINER
====================================
Open this repository in vscode and you can be prompted to rebuild this inside a devcontainer.
This will generate a docker conatiner inside your computer and attach this vscode instance to is. You then can code and debug inside vscode.


If you are on the wamv network you can access the lcm via:
`bot-lcm-tunnel 172.16.154.230`



Useful Commands
--------------------
To get another terminal instance into the docker container use:

`docker exec -it wamv_container /bin/bash`

You will have another terminal instance to interact with in your docker container.


Using ROS
==========================
The docker container is configured to allow ROS access for anyone on the WAM-V Network.
Test this by running 

`roscore`

You will see roscore launch on the wam-v docker container.


To Connect to this ROS instance ensure that you are on the WAM-V Network (Also known as the AUV network). 

On the Client (your own PC)
Run the following. 
```
export ROS_MASTER_URI=http://172.16.154.230:11311
export ROS_HOSTNAME=<YOUR IP ADDRESS>
```
Run `rostopic echo` and you should be able to see some packets from the wamv (`/rosout` and `/rosout_agg`)

Lidar Example
---------------------------
This example will allow you to run the LIDAR from the WAMV and visualise it on a computer connected the the WAM-V network. 

Do the same export commands on the client computer.

To run the Lidar program from the computer run the following on the wam_v docker container.
`roslaunch rslidar_pointcloud rs_lidar_16.launch`


To view the lidar on the client pc:
`rviz`

Then Add the topic to RViz by adding a topic.

Then Ensure to change the base_frame to "rs_lidar"

You should now see the LIDAR on the Screen viewed in RVIZ.


Running the WAM-V Docker Container Locally
=============================================

To run the WAM-V Container locally not much is needed.

Find a workspace ifolder inside your computer and clone the package.

`git clone https://github.com/USYD-RobotX/usyd_robotx_2022.git` 

`cd usyd_robotx_2022`

Build the image and run the container

`docker build -t wamv_image -f wamv_Dockerfile .`

`docker run --network=host -it --name wamv_container wamv_image:latest`

If you are on the WAM-V / AUV network and LCM is running on the system you can acccess the system with the following.

`bot-lcm-tunnel 172.16.154.230`

