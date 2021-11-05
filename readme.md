# ROS Noetic nodes for running the Oak-D camera 
Made for a Raspberry Pi 4, Ubuntu 20.04 and ROS Noetic

The camera is connected to the RPi via the USB3 cable, and only requires ROS Noetic + the camera driver to be installed. (Depthai-ROS)
The base-station laptop requires everything to run, for usefulness sake in debugging, Depthai-ROS is also installed in the ros_ws.


## Install prerequisites:
### Depthai-ROS:
following the normal install instructions from the github repo.
[Depthai ROS](https://github.com/luxonis/depthai-ros)

This builds the depthai-core API, and has the files for the depthai-ros-bridge and examples.

### ORB-SLAM3:

works for me on 20.04//Noetic with the following two patches:

- This Pull request for the CMAKEFILE alterations:
[Pull request to build](https://github.com/RG2806/ORB_SLAM3)

- This change for some bugs in the source code:
[Alterations to some files by matlabbe](https://gist.github.com/matlabbe/f5cb281304a1305b2824a6ce19792e13)

use sudo make install for Pangolin instead of cmake --build . as it causes an error /usr/bin/ld: cannot find -lpangolin

### RTABMAP from source:

Notes taken from GitHub issue about ORB_SLAM and RTAB-Map:
- export ORB_SLAM_ROOT_DIR=/home/...../src/ORB_SLAM3
- git clone https://github.com/introlab/rtabmap.git rtabmap
- cd rtabmap/build
- cmake .. -DWITH_G2O=OFF -DWITH_Qt=OFF
- make
- sudo make install

- cd ~/noetic_ws
- git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros


### rpi_node
add to the ros_ws/src folder:

cd ~/noetic_ws/src
git clone https://github.com/jurriandoornbos/rpi_node.git
cd ..
catkin build




