#!/bin/bash
export ROS_MASTER_URI=http://localhost:$1


roslaunch reachability_space_generation m1n6s200_virtual_robot_demo.launch &
sleep 15

rosrun reachability_space_generation reachability_worker.py mico.yaml