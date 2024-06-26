# MAIA packages
MAIA project codebase.

Runs on ROS Noetic and Ubuntu 20.04

## This repo is a ROS workspace
Create the base workspace folder and clone this repo into it.
```
cd
mkdir maia_ws
cd maia_ws
git clone https://github.com/STAM-ROBO/maia_packages src
```

## Dependencies
You need to install these pkgs before building MAIA
```
sudo apt-get update
sudo apt-get install -y python3-catkin-tools ros-noetic-tf2-sensor-msgs ros-noetic-tf ros-noetic-image-transport ros-noetic-realsense2-camera ros-noetic-angles ros-noetic-laser-geometry libsdl-image1.2-dev libsdl-dev libbullet-dev ros-noetic-tf2-geometry-msgs qt5-default libyaml-cpp-dev ros-noetic-laser-filters
```

The YDlidar-SDK is needed and may be installed in any folder.
```
cd
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir -p ~/YDLidar-SDK/build
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```

## Build MAIA
Now you can build MAIA:
```
cd maia_ws
catkin build
```

## Configure the ROS environment to support remote connection
We don't want the default localhost configuration.

On the navigation PC (192.168.100.80):
```
#tell ROS that the IP address to which the connections are bound
export ROS_IP=192.168.100.80
export ROS_MASTER_URI=http://192.168.100.80:11311
```

On the remote control PC (192.168.100.150):
```
#tell ROS that the IP address to which the connections are bound
export ROS_IP=192.168.100.150
export ROS_MASTER_URI=http://192.168.100.80:11311
```


### Network settings
sudo nano /etc/network/interfaces

### Object Detection Module

This module is responsible for detecting objects from the camera feed, and perform 3D localization (using camera intrinsics and depth maps).

The detection is performed in 2D using a COCO-trained detector. the system currently provides wrappers for YOLOv7 is cloned from the official reposistory (published in CVPR2023) and available either using Pytorch or TensorRT (accelerated)


For installation: download and install pytorch (with cuda support) and see requirements.txt for other dependencies
The detector node subscribes to a camera ROS node which publishes raw rgb, depth and intrinsics topics. 

To start the camera node and the object detector, launch the following command:
```
roslaunch object_detector run_detector.launch debug:=[true/false]
```


