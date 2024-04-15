# MAIA packages
MAIA project codebase.

Runs on ROS Noetic and Ubuntu 22.04

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
sudo apt install python3-catkin-tools
sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera libsdl-image1.2-dev libsdl-dev ros-noetic-tf2-sensor-msgs
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

## Launching the camera node
To start the camera node, run the follwing command:
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
This node publishes several topics obtained from the live-stream of the camera (besides others including the calibration parameters), to which we need to subscribe in other modules.

### Object Detection Module
This module is responsible for detecting objects from the camera feed, and perform 3D localization (using camera intrinsics and depth maps).
The detection is performed in 2D using a COCO-trained detector. the system currently provides wrappers for Faster R-CNN and YOLOv7, as two examples for heavy-weight and a ligher detector. Faster R-CNN is used directly from torchvision, while YOLOv7 is cloned from the official reposistory (published in CVPR2023)

For installation: download and install pytorch (with cuda support) and see requirements.txt for other dependencies
