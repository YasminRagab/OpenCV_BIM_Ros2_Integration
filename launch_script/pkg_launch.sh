#!/usr/bin/bash
cd ~/ptp_ws
. install/setup.bash

ros2 launch usb_cam demo_launch.py &

ros2 launch ros2_aruco myrobot_launch.py &

ros2 run rqt_console rqt_console &
