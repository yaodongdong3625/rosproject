#!/bin/bash
echo "1. run roscore..."
roscore &
sleep 2s

echo "2.run base.launch "
source /home/ferly/project/robot/catkin_ws/devel/setup.bash
roslaunch base_control base.launch &
sleep 2s
echo "3.run sick..."
roslaunch nav sick_tim551.launch &
sleep 2s

echo "4.run locailization..."
roslaunch nav amcl_dwa.launch &
sleep 3s
echo "5.run smoother ..."
roslaunch yocs_velocity_smoother standalone.launch &
sleep 1s

wait
exit 0
