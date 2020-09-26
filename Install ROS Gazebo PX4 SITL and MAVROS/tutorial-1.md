# Install ROS, MAVROS, Gazebo and PX4 SITL

> **Note:** arm64 devices (e.g. NVIDIA Jetson Family) doesn't support PX4 SITL.

> **Note:** This tutorial is only for Ubuntu 18.04 operating systems.

## Install ROS, MAVROS, Gazebo
Official instructions for installing ROS, MAVROS, Gazebo can be found 
[here](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html). However, I had some issues installing it that way so made a combination between that method and [this](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html) method. Finally, I made my own installation [bash script](https://raw.githubusercontent.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/master/Install%20ROS%20Gazebo%20PX4%20SITL%20and%20MAVROS/ubuntu_sim_ros_melodic.sh) which worked properly. So let's start!

Gazebo 9 comes by default by installing ROS Melodic, so we shouldn't worry about installing Gazebo.

1. In a bash shell make a `sudo apt update` and write your password to give sudo permissions:

```
sudo apt update
```

2. Download the script:

```
wget https://raw.githubusercontent.com/aarondigu/ROS-with-PX4-SITL-for-Beginners/master/Install%20ROS%20Gazebo%20PX4%20SITL%20and%20MAVROS/ubuntu_sim_ros_melodic.sh

```

3. Run the script:

```
bash ubuntu_sim_ros_melodic.sh
```

4. This should take a while and by the end you should see a succesfully built catkin workspace.

## Install PX4 SITL
This instructions were obtained from [here](https://dev.px4.io/master/en/setup/building_px4.html) and the [PX4/Avoidance repo](https://github.com/PX4/avoidance).

1. Go to your home directory in a bash shell:

```
cd ~
```

2. Enter the following command to get the very latest version of the `PX4/Firmware`:

```
git clone https://github.com/PX4/Firmware.git --recursive
```

3. After some time downloading the `PX4/Firmware` ge to the Firmware directory:

```
cd ~/Firmware
```

4. Install the PX4 "common" dependencies:

```
./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
```

5. Install the GStreamer plugins for the Gazebo camera:

```
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev
```

6. Build and run your first simulation:

```
make px4_sitl_default gazebo

```

7. When the code stops running and you join Gazebo, you can quit the simulation with `crtl+c`
8. Setup some more Gazebo-related environment variables:

```
. ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default

```
