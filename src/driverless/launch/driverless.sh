#!/bin/bash

gnome-terminal --tab -e "sh /home/seu/projects/littleAnt_ws/src/driverless/launch/rtcm.sh" --tab -e "roslaunch /home/seu/projects/littleAnt_ws/src/path_planning/launch/path_planning.launch" --tab -e "roslaunch /home/seu/projects/littleAnt_ws/src/driverless/launch/driverless.launch" --tab -e "rviz"

