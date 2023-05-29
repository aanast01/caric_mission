# Build the workspace
catkin build;

# Launch the drones

(
    sleep 30;
    wait;
    
    (
        rosservice call /firefly1/traj_gennav/readfile;
        rosservice call /firefly1/traj_gennav/execute_path;
    ) &

    (
        rosservice call /firefly2/traj_gennav/readfile;
        rosservice call /firefly2/traj_gennav/execute_path;
    ) &

    (
        rosservice call /firefly3/traj_gennav/readfile;
        rosservice call /firefly3/traj_gennav/execute_path;
    ) &

    (
        rosservice call /firefly4/traj_gennav/readfile;
        rosservice call /firefly4/traj_gennav/execute_path;
    ) &

    (
        rosservice call /firefly5/traj_gennav/readfile;
        rosservice call /firefly5/traj_gennav/execute_path;
    )
) & \
roslaunch caric_mission run_mbs.launch