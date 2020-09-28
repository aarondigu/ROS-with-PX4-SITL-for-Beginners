# ROS Node for MAVROS Offboard Mode

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

