# ROS Noetic nodes for running the Oak-D camera 
Made for a Raspberry Pi 4, Ubuntu 20.04 and ROS Noetic

## Install prerequisites:
* Depthai-ROS:
following the normal install instructions from the github repo.
[Depthai ROS](https://github.com/luxonis/depthai-ros)

This builds the depthai-core API, and has the files for the depthai-ros-bridge and examples.

* ORB-SLAM3:



(works on 20.04//Noetic with the following two patches:)
- This Pull request for the CMAKEFILE alterations:
[Pull request to build](https://github.com/RG2806/ORB_SLAM3)

- This change for some bugs in the source code:
[Alterations to some files by matlabbe](https://gist.github.com/matlabbe/f5cb281304a1305b2824a6ce19792e13)

use sudo make install for Pangolin instead of cmake --build . as it causes an error /usr/bin/ld: cannot find -lpangolin

* RTABMAP from source:

Notes:
- export ORB_SLAM_ROOT_DIR=/home/...../src/ORB_SLAM3
- git clone https://github.com/introlab/rtabmap.git rtabmap
- cd rtabmap/build
- cmake .. -DWITH_G2O=OFF -DWITH_Qt=OFF
- make
- sudo make install

- cd ~/noetic_ws
- git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
- catkin_make -j4





