#!/usr/bin/bash
. install/setup.bash

kill $(ps -A | grep usb_cam | head -n1 | awk '{print $1;}')
killall robot
killall robot_state_publisher
killall joy_node
killall rviz2
killall rqt_console
kill $(ps -A | grep usb_cam | head -n1 | awk '{print $1;}')
