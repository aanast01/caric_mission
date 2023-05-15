# Simulating for Cooperative Multi-UAV Inspection Challenge

# Prerequisites

* Ubuntu 20.04
* ROS Noetic

# Installations

Building a workspace for the challenge's packages in home directory

```
cd

mkdir -p ws_caric/src;

cd ws_caric/src;

git clone https://github.com/brytsknguyen/caric_mission
git clone https://github.com/brytsknguyen/rotors_simulation
git clone https://github.com/caomuqing/tcc
git clone https://github.com/caomuqing/traj_gennav

catkin build;

source ~/ws_caric/devel/setup.bash

```

# Run the Marina Bay Sands Mission

```
roslaunch caric_mission run_mbs.launch \
& \
(
sleep 3;
roscd caric/scripts/
bash launch_all.sh
)
```