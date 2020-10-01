#!/bin/bash

# close all gazebo simulations
killall gzserver 

source ./launch-common.sh ${1:-empty}

# $1 by default is "empty", the argument can be: baylands, mcmillan_airfield, etc...
roslaunch offb first_mission_debug.launch world:=${1:-empty}