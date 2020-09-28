# ROS Node for MAVROS Offboard Mode

> **NOTE:** The files created in this tutorial can be found [here](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/tree/master/3.%20ROS%20Node%20for%20MAVROS%20Offboard%20Mode).

In this tutorial you will learn to hover the iris drone using a ROS Node and [ROS Wrappers](https://dev.px4.io/master/en/simulation/ros_interface.html#launching-gazebo-with-ros-wrappers), but also executing PX4 SITL, Gazebo and MAVROS. This tutorial is based on this [great tutorial](https://darienmt.com/autonomous-flight/2018/11/25/px4-sitl-ros-example.html), but I made some modifications to use it with any of the [PX4/sitl_gazebo/worlds](https://dev.px4.io/master/en/simulation/gazebo_worlds.html) available. 

So, without further ado, let's begin!

## Creating a ROS package

1. Launch your catkin workspace, make a directory named `src` and move into it:

```
cd ~/catkin_ws
mkdir src
cd src
```

2. Now let's create the **offb** package, which is dependant on std_msgs, roscpp, geometry_msgs and mavros_msgs:

```
catkin_create_pkg offb std_msgs roscpp geometry_msgs mavros_msgs
```

3. Create the file `offb_node.cpp` in the `src` directory of the package you just created: 

```
touch offb/src/offb_node.cpp
```

4. Open `offb_node.cpp` and paste the code from this [site](https://dev.px4.io/master/en/ros/mavros_offboard.html), in which the code is already explained in detail.

5. Open the `CMakeList.txt` file of the package you just created (/offb/CMakeList.txt) and uncomment the lines:

```
add_executable(${PROJECT_NAME}_node src/offb_node.cpp)

# and 

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
```

6. Build the package from the workspace source route (~/catkin_ws):

```
cd ~/catkin_ws
catkin build
```

## Creating a launch file

7.  We need a launch file to launch PX4, Gazebo, MAVROS and our node (offb). The launchfile [mavros_posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/mavros_posix_sitl.launch) inside the Firmware already launches PX4, Gazebo, MAVROS and spawns the vehicle. We will include this launchfile in our launchfile and we only need to run our node (offb), so let's create our launchfile:

```
cd ~/catkin_ws
mkdir -p offb/launch
touch offb/launch/offb.launch
```

8. Open the launchfile `offb.launch` and paste the following there:
```
<?xml version="1.0"?>
<launch>

<!-- launches MAVROS, PX4 SITL, Gazebo environment,  spawns vehicle and runs offb_node -->
    
    <!-- Gazebo World config -->    
    <arg name="world" default="empty"/>

    <!-- MAVROS, PX4 SITL and Gazebo -->
    <include file="$(find px4)/launch/mavros_posix_sitl.launch">
        <arg name="world" value="$(find mavlink_sitl_gazebo)/worlds/$(arg world).world"/>
    </include>
    
    <!-- offb_node-->
    <node name="offb" pkg="offb" type="offb_node" output="screen"/>
</launch>
```

## Creating bash scripts to run our application

9. Now we have to run this launchfile, but what are those "px4" and "mavlink_sitl_gazebo" packages? They are actually ~/Firmware and ~/Firmware/Tools/sitl_gazebo directories, respectively; but we want ROS to see them as packages. We will do that in a bash script. Let's create a new `src` directory in our home and create the `launch-common.sh` script: 
```
cd ~
mkdir src
touch launch-common.sh
```

10. Open the file and paste the following there:

```
#!/bin/bash


cd ~/Firmware

DONT_RUN=1 make px4_sitl_default gazebo___${1:-empty}

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

In this file, we use [ROS Wrappers](https://dev.px4.io/master/en/simulation/ros_interface.html#launching-gazebo-with-ros-wrappers) to make ROS see those paths as packages.

11. Now, we need another bash file to run `launch-common.sh` and also the launchfile `offb.launch`. Let's create it:

```
cd ~/src
touch launch-offb.sh
chmod +x launch-offb.sh
```

10. Open the file and paste the following there:

```
#!/bin/bash

# close all gazebo simulations
killall gzserver 

source ./launch-common.sh

# $1 by default is "empty", the argument can be: baylands, mcmillan_airfield, etc...
roslaunch offb offb.launch world:=${1:-empty}
```

This script needs an argument when we run it, that argument is the world we want to simulate with our drone. Available worlds are [PX4/sitl_gazebo/worlds](https://dev.px4.io/master/en/simulation/gazebo_worlds.html). To run this script we need the command: `./launch-offb.sh <world>`. If no argument is given, the world `empty` will be used by default.

11. Finally, let's fly our drone! Let's hover our drone in the McMillan Airfield world. Open a new terminal and run this:
```
cd ~/src
./launch-offb.sh mcmillan_airfield
```

12. It will take a couple of seconds, but hopefully, Gazebo will start, and you will see the drone hovering!