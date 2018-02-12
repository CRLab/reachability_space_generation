#!/bin/bash
export ROS_MASTER_URI=http://localhost:$1

roslaunch reachability_space_generation barrett_workspace.launch &
sleep 15

rosrun reachability_space_generation reachability_worker.py barrett.yaml