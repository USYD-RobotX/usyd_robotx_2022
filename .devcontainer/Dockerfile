# Start with default docker image for ROS1 noetic over Ubuntu 20.04 Focal.
FROM osrf/ros:noetic-desktop-full 

SHELL ["/bin/bash", "-c"] # Specify a bash shell for run commands.

# Install catkin & dependencies:
RUN apt-get update && apt-get install -y \
    python3-catkin-pkg \
    ros-noetic-catkin \
    python3-catkin-tools 


# Add packages to install below (don't forget the backslash).
RUN apt-get update && apt-get install -y \    
    git \
    nano \
    net-tools \
    screenfetch \ 
    tree \
    wget

RUN apt-get update && apt-get install -y \    
    build-essential \
    libglib2.0-dev \
    cmake \
    python-all-dev\
    python3-pip

RUN apt-get update && sudo apt-get install python2-minimal

RUN pip3 install lcm

RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 2


# installing lcm
RUN git clone https://github.com/lcm-proj/lcm.git

RUN export LCM_INSTALL_PREFIX=/usr/local
RUN export LCM_LIBRARY_DIR=/usr/local/lib
RUN echo $LCM_LIBRARY_DIR > /etc/ld.so.conf.d/lcm.conf

RUN PYTHON_VERSION=$(python -c "import sys; print(\"%s.%s\" % sys.version_info[:2])")

RUN PYTHON_USER_SITE=$(python -m site --user-site)

RUN echo "$LCM_LIBRARY_DIR/python$PYTHON_VERSION/site-packages" > $PYTHON_USER_SITE/lcm.pth

RUN cd lcm && mkdir build && cd build && cmake .. && make && sudo make install

RUN cd /lcm/lcm-python && python2 setup.py install

# # Installing the LCM tunnel, keyrick and repo. Commands as gotten from Trello
RUN wget http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-keyring_focal_all.deb \
         http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-repo_focal_all.deb
RUN dpkg -i acfrmarine-*.deb
RUN apt-get update && apt install bot2-all -y


RUN apt-get update && apt install ros-noetic-pcl-ros libpcap-dev vim -y

RUN wget https://downloads.alliedvision.com/Vimba64_v6.0_Linux.tgz && tar -xvf Vimba64_v6.0_Linux.tgz && cd Vimba_6_0/VimbaGigETL && ./Install.sh

RUN mkdir -p /rx_ws/src/usyd_robotx_2022

RUN cd /rx_ws/src && git clone https://github.com/astuff/avt_vimba_camera.git

RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc

RUN echo "[[ -f /rx_ws/devel/setup.bash ]] && source /rx_ws/devel/setup.bash" >> /root/.bashrc

ENV export GENICAM_GENTL64_PATH=$GENICAM_GENTL64_PATH:"/Vimba_6_0/VimbaGigETL/CTI/x86_64bit"

# RUN echo "alias alias_name='/rx_ws/src/LCM/pyspy'" >> /root/.bashrc
# export GENICAM_GENTL64_PATH=$GENICAM_GENTL64_PATH:"/Vimba_6_0/VimbaGigETL/CTI/x86_64bit
# roslaunch usydrx_vision start_all_cameras.launch
# ENV ROS_MASTER_URI=http://172.16.154.230:11311

# ENV ROS_HOSTNAME=172.16.154.230
# ADD . /rx_ws/src/usyd_robotx_2022

# RUN source /opt/ros/noetic/setup.bash && cd /rx_ws && catkin_make

# RUN source /rx_ws/devel/setup.bash

# RUN chmod +x /rx_ws/src/usyd_robotx_2022/LCM/acfr/pyspy

# RUN ln -s /rx_ws/src/usyd_robotx_2022/LCM/acfr/pyspy /usr/local/bin/pyspy


# # Copy pyspy to docker container
# COPY pyspy /usr/local/bin/

# RUN apt-get update && apt-get install -y python

# COPY lcm /usr/lib/python2.7/dist-packages/lcm 




# RUN apt-get autoremove -y

# WORKDIR /robotxDevEnv
# RUN git clone https://github.com/USYD-RobotX/robotx_2022 

# ### Set up the Heartbeat ROS Node. ###

# # Skip all caching for the catkin workstace build (comment to cache).
# #ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache

# # Source the ROS distro setup file.
# RUN . /opt/ros/noetic/setup.bash

# # Create and move to the catkin workspace.
# RUN mkdir -p robotx_ws/src
# WORKDIR /robotx_ws

# # Clone the comms & heartbeat scripts from GitHub.
# WORKDIR src
# RUN git clone https://github.com/USYD-RobotX/wamv_comms
# WORKDIR /robotx_ws

# # Initialise & build the catkin workspace.
# RUN catkin init
# RUN catkin config --extend /opt/ros/noetic
# RUN catkin build

# RUN . devel/setup.bash

# WORKDIR ..

