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
sudo apt-get install -y nano spacenavd ros-noetic-spacenav-node python3-catkin-tools ros-noetic-realsense2-camera libsdl-image1.2-dev libsdl-dev libbullet-dev ros-noetic-tf2-geometry-msgs qt5-default libyaml-cpp-dev ros-noetic-laser-filters ros-noetic-cv-camera ros-noetic-camera-calibration
wget -O torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl pyserial
wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt
```

## Install usb_camera (NVIDIA version is broken)
sudo nano /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
->>>Comment the line - deb https://repo.download.nvidia.com/jetson/ffmpeg r36.3 main
sudo add-apt-repository ppa:savoury1/ffmpeg4
sudo apt-get update
sudo apt remove ffmpeg
sudo apt install ffmpeg

## Install and build torchvision
```
cd
git clone https://github.com/pytorch/vision.git -b release/0.16
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
cd vision
export BUILD_VERSION=0.16.0


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


