# Docker commands to build the container for development.
docker build -t wamv_image -f wamv_Dockerfile .
docker rm wamv_container
docker run --network=host -it --name wamv_container wamv_image:latest


# Setting the ROS_MASTER on the master
export ROS_MASTER_URI=http://172.16.154.230:11311
export ROS_HOSTNAME=172.16.154.230




# Setting the ROS_CLIENT
export ROS_MASTER_URI=http://172.16.154.230:11311
export ROS_HOSTNAME=172.16.154.229


# Connecting to a server using bot-lcm-tunnel
bot-lcm-tunnel 172.16.154.230