# First Mission with MAVROS and PX4 in Offboard Mode

> **NOTE:** The files created in this tutorial can be found [here](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/tree/master/4.%20First%20Mission%20with%20ROS%20PX4%20SITL%20and%20Gazebo).

This tutorial is very similar to the [last tutorial](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/3.%20ROS%20Node%20for%20MAVROS%20Offboard%20Mode/tutorial-3.md), but instead of just hovering your drone, you will learn how to define a mission (set of "goal" points for the drone to reach) for the iris drone using a ROS Node and [ROS Wrappers](https://dev.px4.io/master/en/simulation/ros_interface.html#launching-gazebo-with-ros-wrappers), but also executing PX4 SITL, Gazebo and MAVROS. 

So, without further ado, let's begin!

## Updating the "offb" ROS package

We will first add a new node to our package, which will help us to make our first mission.

1. Launch the `src` directory of the **offb** package and create the `first_mission_node.cpp` script:

```
cd cd ~/catkin_ws/src/offb/src
touch first_mission_node.cpp
```

2. Open `first_mission_node.cpp` and paste [this code](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/4.%20First%20Mission%20with%20ROS%20PX4%20SITL%20and%20Gazebo/first_mission_node.cpp).

3. Open the `CMakeList.txt` file of the **offb** package (/offb/CMakeList.txt) and add the lines:

```
add_executable(first_mission_node src/first_mission_node.cpp)

# and 

target_link_libraries(first_mission_node 
 ${catkin_LIBRARIES}
)
```
  Your `CMakeList.txt` file should be like [this one](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/4.%20First%20Mission%20with%20ROS%20PX4%20SITL%20and%20Gazebo/CMakeLists.txt).

4. Build the package from the workspace source route (~/catkin_ws):

```
cd ~/catkin_ws
catkin build
```

## Creating a launch file

7.  We need a launch file to launch PX4, Gazebo, MAVROS and our new node (first_mission). The launchfile [mavros_posix_sitl.launch](https://github.com/PX4/Firmware/blob/master/launch/mavros_posix_sitl.launch) inside the Firmware already launches PX4, Gazebo, MAVROS and spawns the vehicle. We will include this launchfile in our launchfile and we only need to run our node (first_mission), so let's create our launchfile:

```
cd ~/catkin_ws/src/offb/launch
touch first_mission.launch
```

8. Open the launchfile `first_mission.launch` and paste the following there:
```
<?xml version="1.0"?>
<launch>
<!-- MAVROS posix SITL environment launch script -->
<!-- launches MAVROS, PX4 SITL, Gazebo environment,  spawns vehicle and first_mission_node -->


    <!-- Gazebo World config -->    
    <arg name="world" default="empty"/>

    <!-- MAVROS, PX4 SITL and Gazebo -->
    <include file="$(find px4)/launch/mavros_posix_sitl.launch">
        <arg name="world" value="$(find mavlink_sitl_gazebo)/worlds/$(arg world).world"/>
    </include>
    

    <!-- first_mission_node-->
    <node name="offboard" pkg="offb" type="first_mission_node" output="screen"/>
</launch>
```
  Your `first_mission.launch` file should be like [this one](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/4.%20First%20Mission%20with%20ROS%20PX4%20SITL%20and%20Gazebo/first_mission.launch).

## Creating bash scripts to run our application

9. Now we have to run this launchfile, but what are those "px4" and "mavlink_sitl_gazebo" packages? They are actually ~/Firmware and ~/Firmware/Tools/sitl_gazebo directories, respectively; but we want ROS to see them as packages. We will do that in a bash script. Let's launch `src` directory in our home and create the `launch-common.sh` script... **But wait!!** we already have that file, so we will just re-use it.

  Your `launch-common.sh` file should be like [this one](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/3.%20ROS%20Node%20for%20MAVROS%20Offboard%20Mode/launch-common.sh).

11. Now, we need another bash file to run `launch-common.sh` and also the launchfile `first_mission.launch`. Let's create it:

```
cd ~/src
touch launch-first_mission.sh
chmod +x launch-first_mission.sh
```

10. Open the file and paste the following there:

```
#!/bin/bash

# close all gazebo simulations
killall gzserver 

source ./launch-common.sh ${1:-empty}

# $1 by default is "empty", the argument can be: baylands, mcmillan_airfield, etc...
roslaunch offb first_mission.launch world:=${1:-empty}
```
  Your `launch-first_mission.sh` file should be like [this one](https://github.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/blob/master/4.%20First%20Mission%20with%20ROS%20PX4%20SITL%20and%20Gazebo/launch-first_mission.sh).

This script needs an argument when we run it, that argument is the world we want to simulate with our drone. Available worlds are [PX4/sitl_gazebo/worlds](https://dev.px4.io/master/en/simulation/gazebo_worlds.html). To run this script we need the command: `./launch-first_mission.sh <world>`. If no argument is given, the world `empty` will be used by default.

11. Finally, let's fly our drone! Let's take our drone for its first mission in the McMillan Airfield world. Open a new terminal and run this:
```
cd ~/src
./launch-first_mission.sh mcmillan_airfield
```

12. It will take a couple of seconds, but hopefully, Gazebo will start, and you will see the drone making a square in the air! Something similar to this [**video**](https://youtu.be/3lI75DczVtw).
