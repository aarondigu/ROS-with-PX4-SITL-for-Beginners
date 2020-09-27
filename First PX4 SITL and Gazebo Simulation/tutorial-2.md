# First PX4 SITL Simulation with ROS and Gazebo


## First PX4 SITL Simulation without MAVROS
For this simulation without MAVROS, we will literally follow the official [tutorial](https://dev.px4.io/master/en/simulation/ros_interface.html) from PX4.

1. Change directory to the cloned PX4/Firmware directory:
```
cd ~/Firmware
```

2. Make the PX4 simulation with Gazebo without running it:
```
DONT_RUN=1 make px4_sitl_default gazebo
```

3. Setup some environment variables:
```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
```

4. Add the Firmware directory and gthe gazebo related directory to ROS_PACKAGE_PATH so that ROS can start px4 and mavlink_sitl_gazebo:
``` 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
``` 

5. Launch the simulation using ROS: 
``` 
roslaunch px4 posix_sitl.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/baylands.world

```
The world by default is `empty.world`, but using the argument `world` we can choose which world we want to simulate. For this example I used `bandalays.world` but you can use any of the [PX4/sitl_gazebo/worlds](https://dev.px4.io/master/en/simulation/gazebo_worlds.html) available. You just need to replace `bandalays.world` for `<world_name>.world`.

6. Once the simulation is running, press the "Enter" key and you should see `pxh >`.

7. Type "help" to check all commands available:

> **NOTE:** In Gazebo, you can follow the drone with the camera by right-clicking on it and checking "Follow".

```
pxh > help
```

8. Change the flight mode to "offboard":
```
pxh > commander mode offboard
```

9. Take off the drone:
```

pxh > commander takeoff
```

10. Land the drone:
```

pxh > commander land

```

> **IMPORTANT:** Don't stop your simulation yet!

This was a very basic example because we won't be using these commands anymore. It results more interesting and easier to use MAVROS commands, as we are going to see next.

## Simulation with MAVROS

>**NOTE:** It is assumed that you are already familiarized with ROS, you can check the official ROS tutorials on the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials).

We are going to use some MAVROS commands which are very handful. You can read more about them [here](http://wiki.ros.org/mavros#Nodes).

1. Open a new terminal and launch MAVROS to connect the simulation with ROS:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

2. In a new terminal, echo the `/mavros/state` topic, which will tell us about the current state of the drone:
```
rostopic echo /mavros/state
```

3. In a new terminal, publish in the `/mavros/setpoint_position/local` topic, which will tell PX4 the position and orientation we want to reach. In this particular case, we want the drone to modify only its altitude to 3.0 meters, without any rotations. This message will be published at a rate of 10 Hz.
```
rostopic pub -r 10  /mavros/setpoint_position/local geometry_msgs/PoseStamped "{header:  {seq: 0, stamp: {secs: 0, nsecs: 0}}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

4. In a new terminal, use MAVROS mavsafety to arm the drone:
```
rosrun mavros mavsafety arm
```

5. Use MAVROS mavsys command to change the flight mode to "offboard", be fast because you only have a few seconds of inactivity before PX4 automatically returns to "manual" mode after arming the drone:
```
rosrun mavros mavsys  mode -c OFFBOARD
```

6. Use MAVROS mavsetp to set a local point the drone has to reach:
```
rosrun mavros mavsetp local 0 4 3 0 0 0
```

6. Finally, use MAVROS mavcmd to land the drone:
```
rosrun mavros mavcmd land -v 0 0 0 0
```

