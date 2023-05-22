# CARIC@OneStop: 5-minute setup for Cooperative Aerial Inspection Challenge

# Prerequisites

* Ubuntu 20.04
* ROS Noetic
* Gazebo 11 and other dependencies (30min+, consider grabbing a kopi)

```
sudo apt-get update && sudo apt upgrade
sudo apt-get install python-wstool python-catkin-tools \
                     protobuf-compiler libgoogle-glog-dev \
                     ros-$ROS_DISTRO-control-toolbox \
                     ros-$ROS_DISTRO-octomap-msgs \
                     ros-$ROS_DISTRO-octomap-ros
sudo apt-get install ros-noetic-gazebo*
```

# Installations

Building a workspace for the challenge's packages in home directory

```
cd
mkdir -p ws_caric/src;
catkin_init_workspace;
cd ws_caric/src;

git clone https://github.com/ntu-aris/caric_mission
git clone https://github.com/ntu-aris/rotors_simulator
git clone https://github.com/ntu-aris/tcc
git clone https://github.com/ntu-aris/traj_gennav
git clone https://github.com/ntu-aris/velodyne_simulator
catkin build;

source ~/ws_caric/devel/setup.bash

```

# Run the Marina Bay Sands Mission

```
roscd caric_mission/scripts;
bash launch_mbs.sh
```
