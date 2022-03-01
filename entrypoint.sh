#!/bin/bash

source /opt/ros/galactic/setup.bash

ROS_FLAGS=""
if [ ${SIMULATION+x} != "" ]; then
    ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

ros2 launch control_interface control_interface.py ${ROS_FLAGS}
