#!/bin/bash

# close all gazebo simulations
killall gzserver 

source ./launch-common-crone.sh ${1:-empty}

# $1 by default is "empty", the argument can be: baylands, mcmillan_airfield, etc...
roslaunch offb cr_mission.launch world:=${1:-empty}