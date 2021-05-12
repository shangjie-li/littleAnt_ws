#!/bin/bash

gnome-terminal \
--tab -e 'sh ~/projects/littleAnt_ws/src/drivers/rtcm3.2/main' \
--tab -e 'roslaunch ~/projects/littleAnt_ws/src/path_planning/launch/path_planning.launch' \
--tab -e 'roslaunch ~/projects/littleAnt_ws/src/driverless/launch/driverless.launch' \
--tab -e 'rviz'
