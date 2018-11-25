#!/bin/bash
# roscore

gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
sleep 1s

#diff_controller started 
gnome-terminal -t "diff_controller started " -x bash -c "roslaunch base base_diff_drive_common.launch;exec bash;"
sleep 1s

#move base started
gnome-terminal -t "move base " -x bash -c "roslaunch nav fake_hqrobot.launch;exec bash;"
sleep 1s

#amlcl simulation mode
gnome-terminal -t "amcl" -x bash -c "roslaunch nav ls.launch;exec bash;"
sleep 1s


# simulation mode
gnome-terminal -t "gmapping" -x bash -c "roslaunch nav gmapping_demo.launch;exec bash;"
sleep 1s

#topic relay
gnome-terminal -t "topic cmd_vel relay" -x bash -c "rosrun topic_tools relay /cmd_vel /mobile_base_controller/cmd_vel;exec bash;"
sleep 1s
gnome-terminal -t "topic odom relay" -x bash -c "rosrun topic_tools relay /mobile_base_controller/odom /odom;exec bash;"
sleep 1s

#view navigation mode
gnome-terminal -t "view_rviz" -x bash -c "roslaunch nav view_navigation.launch;exec bash;"
sleep 1s


rosrun mcu mcu_node device:=/dev/ttyUSB0 type:=toMCU board:=foot&
sleep 3s
rosrun mcu mcu_node device:=/dev/ttyUSB0 type:=toMCU board:=foot&

