#!/usr/bin/env bash
#set IP ADDRESS of this machine
export ROS_IP=192.168.100.80

#set IP ADDRESS of roscore
export ROS_MASTER_URI=http://192.168.100.80:11311

cd 
cd maia_ws
source devel/setup.bash

roslaunch maia_powerchair maia_launch.launch 
