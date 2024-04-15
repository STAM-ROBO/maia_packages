# maia_ws
MAIA project codebase

## Get MAIA

```
git clone https://github.com/STAM-ROBO/maia_ws
```


## Dependencies
You need to install these pkgs before building MAIA
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-kinetic-tf2-sensor-msgs
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
To build MAIA:
```
cd maia_ws
catkin_make
```

## Launching the camera node
To start the camera node, run the follwing command:
```
roslaunch realsense2_camera rs_camera.launch
```
This node publishes several topics obtained from the live-stream of the camera (besides others including the calibration parameters), to which we need to subscribe in other modules.

### Object Detection Module
This module is responsible for detecting objects from the camera feed, and perform 3D localization (using camera intrinsics and depth maps).
The detection is performed in 2D using a COCO-trained detector. the system currently provides wrappers for Faster R-CNN and YOLOv7, as two examples for heavy-weight and a ligher detector. Faster R-CNN is used directly from torchvision, while YOLOv7 is cloned from the official reposistory (published in CVPR2023)
