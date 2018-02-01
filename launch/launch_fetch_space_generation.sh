#!/bin/bash
export ROS_MASTER_URI=http://localhost:$1

roslaunch reachability_space_generation fetch_moveit.launch &
sleep 15

rosrun reachability_space_generation reachability_worker.py fetch22.yaml