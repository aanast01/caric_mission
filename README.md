# Simulating for Cooperative Aerial Inspection Challenge

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
git clone https://github.com/brytsknguyen/rotors_simulator
git clone https://github.com/caomuqing/tcc
git clone https://github.com/caomuqing/traj_gennav

catkin build;

source ~/ws_caric/devel/setup.bash

```

# Run the Marina Bay Sands Mission

```

(
roscd caric_mission/scripts/
sleep 10;
wait;
echo LAUNCHING
bash launch_all.sh
) & \
roslaunch caric_mission run_mbs.launch

```
