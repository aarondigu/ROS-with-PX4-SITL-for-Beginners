#Install ROS, MAVROS, Gazebo and PX4 SITL

>Note: arm64 devices (e.g. NVIDIA Jetson Family) doesn support PX4 SITL.

>Note: This tutorial is only for Ubuntu 18.04 operating systems.

##Install ROS, MAVROS, Gazebo
Official instructions for installing ROS, MAVROS, Gazebo can be found [here]{https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html}. However, I had some issues installing it that way so made a combination between that method and [this]{https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html} method. Finally, I made my own installation [bash file]{} which worked properly.