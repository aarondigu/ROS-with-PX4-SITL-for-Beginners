# First PX4 SITL Simulation with ROS and Gazebo


## First PX4 SITL Simulation without MAVROS

```
cd Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/baylands.world

```

```
En gazebo clic derecho en el dron, follow
Press enter
pxh > help
pxh > commander mode offboard
pxh > commander takeoff
pxh > commander land

```

## Simulation with MAVROS
```
New terminal
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

New terminal
rostopic echo /mavros/state

New terminal
rostopic pub -r 10  /mavros/setpoint_position/local geometry_msgs/PoseStamped "{header:  {seq: 0, stamp: {secs: 0, nsecs: 0}}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"

New terminal 
rosrun mavros mavsys  mode -c OFFBOARD
rosrun mavros mavsafety arm




```
