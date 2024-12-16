#!/usr/bin/env bash
#trigger udev rule to assign symbolic device names to physical USB devices
#rules defined by Davide Di Gloria into /etc/udev/rules.d/99-usb-serial.rules 
#sudo udevadm trigger

#get permissions for ttys
#sudo chmod a+rw /dev/usb_lidar
#sudo chmod a+rw /dev/usb_motors
sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyACM0


#again, set the IP address of roscore
export ROS_IP=172.17.24.63

#set IP address of roscore
export ROS_MASTER_URI=http://172.17.24.63:11311

cd 
cd maia_ws
source devel/setup.bash
roslaunch maia_powerchair maia-nav.launch 
