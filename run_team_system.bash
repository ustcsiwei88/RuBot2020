#!/usr/bin/env bash
/opt/ros/${ROS_DISTRO}/setup.bash
~/rubot/install/setup.bash
cd ~/rubot
source devel/setup.bash
# sleep 16s
rosrun rubot rubot_node
