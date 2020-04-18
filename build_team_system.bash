!/usr/bin/env bash
. /opt/ros/${ROS_DISTRO}/setup.bash
cd ~
git clone https://github.com/ustcsiwei88/RuBot2020.git rubot
cd rubot
git submodule update --init
catkin build