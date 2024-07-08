#!/usr/bin/env bash
#set IP ADDRESS of this machine

#trigger udev rule tu assign usb devisec names
sudo udevadm trigger

#get rw permission
sudo chmod a+rw /dev/usb_lidar
sudo chmod a+rw /dev/usb_motors
export ROS_IP=192.168.100.207

#set IP ADDRESS of roscore
export ROS_MASTER_URI=http://192.168.100.207:11311

cd 
cd maia_ws
source devel/setup.bash

roslaunch maia_powerchair maia-nav.launch 
