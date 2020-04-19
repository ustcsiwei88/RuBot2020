#!/usr/bin/env bash

YELLOW='\033[0;33m'
GREEN='\033[0;32m'
NOCOLOR='\033[0m'

# Prepare ROS
echo -e "${YELLOW}---Sourcing ROS${NOCOLOR}"
. /opt/ros/${ROS_DISTRO}/setup.bash
echo -e "${YELLOW}---Sourcing ARIAC${NOCOLOR}"
. /home/ariac-user/ariac_ws/devel/setup.bash


# Install the necessary dependencies for getting the team's source code
# Note: there is no need to use `sudo`.
# echo -e "${YELLOW}---Installing necessary dependencies${NOCOLOR}"
# apt-get update
# apt-get install -y wget unzip


# Create a catkin workspace
echo -e "${YELLOW}---Creating workspace${NOCOLOR}"
cd /root
git clone https://github.com/ustcsiwei88/RuBot2020 rubot
catkin_make