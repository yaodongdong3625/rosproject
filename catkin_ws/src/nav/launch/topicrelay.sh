#!/bin/bash
# roscore
#topic relay
gnome-terminal -t "topic cmd_vel relay" -x bash -c "rosrun topic_tools relay /cmd_vel /mobile_base_controller/cmd_vel;exec bash;"
sleep 1s
gnome-terminal -t "topic odom relay" -x bash -c "rosrun topic_tools relay /mobile_base_controller/odom /odom;exec bash;"
sleep 1s
