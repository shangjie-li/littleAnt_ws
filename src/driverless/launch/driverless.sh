#!/bin/bash

gnome-terminal --tab -e "roslaunch /home/seu/projects/littleAnt_ws/src/v2x/obu/obu_to_xy/launch/obu_fusion_start.launch" --tab -e "sh /home/seu/projects/littleAnt_ws/src/driverless/launch/rtcm.sh" --tab -e "roslaunch /home/seu/projects/littleAnt_ws/src/path_planning/launch/path_planning.launch" --tab -e "roslaunch /home/seu/projects/littleAnt_ws/src/driverless/launch/driverless.launch" --tab -e "rviz"

