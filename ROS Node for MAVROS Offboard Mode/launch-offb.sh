#!/bin/bash

source ./launch-common.sh

roslaunch offb offb.launch world:=${1:-empty}