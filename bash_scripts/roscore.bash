#!/bin/bash

#Saving the arguments given while executing the script
ROS_HOSTNAME=$1 #The numbers indicate the position of the given argument
ROS_MASTER_URI=$2

#Access to Noetic Workspace
cd $HOME/rover_ws

#Seting up ROS1 enviroment
catkin_make

#Sourcing ROS1 setup.bash script
source devel/setup.bash

#Export ROS_HOSTNAME and ROS_MASTER_URI so it can works from others machines
export ROS_HOSTNAME=$ROS_HOSTNAME
export ROS_MASTER_URI=$ROS_MASTER_URI

#Executing ROS1 roscore
gnome-terminal -- bash -c "echo 'Running rover_setup.bash'; $HOME/rover_ws/bash_scripts/rover_setup.bash "$ROS_HOSTNAME" "$ROS_MASTER_URI"; exec bash"

#Launching roscore
roscore