#!/bin/bash

gnome-terminal -x bash -c "cd /home/seu/projects/littleAnt_ws/src/drivers/rtcm3.2 && ./main; exec bash"
sleep 1

gnome-terminal -x bash -c "roslaunch /home/seu/projects/littleAnt_ws/src/path_planning/launch/path_planning.launch; exec bash"
sleep 1

gnome-terminal -x bash -c "roslaunch /home/seu/projects/littleAnt_ws/src/driverless/launch/driverless.launch; exec bash"
sleep 1

gnome-terminal -x bash -c "rviz; exec bash"

