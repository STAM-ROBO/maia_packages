#!/usr/bin/env bash
#set IP ADDRESS of this machine

sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyACM0
export ROS_IP=192.168.100.207

#set IP ADDRESS of roscore
export ROS_MASTER_URI=http://192.168.100.207:11311

cd 
cd maia_ws
source devel/setup.bash

roslaunch maia_powerchair maia-nav.launch 
