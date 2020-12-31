#!/bin/bash


cd ~/Firmware

DONT_RUN=1 make px4_sitl_default gazebo_crone__${1:-empty}

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo