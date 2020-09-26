# Install ROS, MAVROS, Gazebo and PX4 SITL

> Note: arm64 devices (e.g. NVIDIA Jetson Family) doesn support PX4 SITL.

> Note: This tutorial is only for Ubuntu 18.04 operating systems.

## Install ROS, MAVROS, Gazebo
Official instructions for installing ROS, MAVROS, Gazebo can be found 
[here](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html). However, I had some issues installing it that way so made a combination between that method and [this](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html) method. Finally, I made my own installation [bash script](https://raw.githubusercontent.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/master/INSTALL-ROS-Gazebo-PX4-SITL-and-MAVROS/ubuntu_sim_ros_melodic.sh) which worked properly. So let's start!

Gazebo 9 comes by default by installing ROS Melodic, so we shouldn't worry about installing Gazebo.

1. In a bash shell make a `sudo apt update` and write your password to give sudo permissions:

```
sudo apt update
```

2. Download the script:

```
wget https://raw.githubusercontent.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/master/INSTALL-ROS-Gazebo-PX4-SITL-and-MAVROS/ubuntu_sim_ros_melodic.sh

```

3. Run the script:

```
bash ubuntu_sim_ros_melodic.sh
```

4. This should take a while and by the ending you should see a succesfully built catkin workspace.