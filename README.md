# A module of Cooperative Aerial Inspection Challenge

Please find the instructions at [https://ntu-aris.github.io/caric](https://ntu-aris.github.io/caric)


## Install the dependencies
###. Ubuntu 20.04 + ROS Noetic
Please install the neccessary dependencies by the following commands:

### Update the system
```shell
sudo apt-get update && sudo apt upgrade ;
```

### Install some tools and dependencies
```shell
sudo apt-get install python3-wstool python3-catkin-tools python3-empy \
                     protobuf-compiler libgoogle-glog-dev \
                     ros-$ROS_DISTRO-control-toolbox \
                     ros-$ROS_DISTRO-octomap-msgs \
                     ros-$ROS_DISTRO-octomap-ros \
                     ros-$ROS_DISTRO-mavros \
                     ros-$ROS_DISTRO-mavros-msgs \
                     ros-$ROS_DISTRO-rviz-visual-tools \
                     ros-$ROS_DISTRO-gazebo-plugins \
                     python-is-python3;
```

## Install gazebo 11 (default for Ubuntu 20.04)
```shell
sudo apt-get install ros-noetic-gazebo* ;
```

### Install the CARIC packages
Once the dependencis have been installed, please create a new workspace for CARIC, clone the necessary packages into it, and compile:

### Create the workspace
```shell
mkdir -p ~/ws_caric/src
cd ~/ws_caric/src

## Download the packages:
## Manager node for the mission
git clone https://github.com/aanast01/caric_mission

## Simulate UAV dynamics and other physical proccesses
git clone https://github.com/aanast01/rotors_simulator

## GPU-enabled lidar simulator, modified from: https://github.com/lmark1/velodyne_simulator
git clone https://github.com/aanast01/velodyne_simulator

## Converting the trajectory setpoint to rotor speeds
git clone https://github.com/aanast01/unicon

## To generate an trajectory based on fixed setpoints. Only used for demo, to be replaced by user's inspection algorithms
git clone https://github.com/aanast01/traj_gennav

## Clone the KIOS Team Solution, Winning solution of CARIC 2023 (Optional) (Not Yet Public)
git clone https://github.com/aanast01/caric_cdc_2023.git

## Build the workspace
cd ~/ws_caric/
catkin build
```

The compilation may report errors due to missing depencies or some packages in CARIC are not yet registered to the ros package list. This can be resolved by installing the missing dependencies (via sudo apt install <package> or sudo apt install ros-$ROS_DISTRO-<ros_package_name>)). Please try catkin build again a few times to let all the compiled packages be added to dependency list.
