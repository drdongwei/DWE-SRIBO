#!/bin/bash
# this file is used for estimation onboard
# start sribo_onboard 
gnome-terminal --geometry 150x100 -- bash -c "roslaunch sribo_onboard.launch" & sleep 1

# start rosbag record
gnome-terminal --geometry 150x100+150 -- bash -c "bash rosbag_record.sh" & sleep 1
