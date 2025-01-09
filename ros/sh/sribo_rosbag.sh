#!/bin/bash
# this file is used for estimation with recorded rosbag
# start sribo_rosbag 
gnome-terminal --geometry 110x30+0+0 -- bash -c "roslaunch sribo sribo_rosbag.launch" & sleep 1

# play rosbag
gnome-terminal --geometry 80x30+1500+0 -- bash -c "roslaunch sribo bag_play.launch" & sleep 1

# start rosbag record
# gnome-terminal --geometry 80x30+1500+800 -- bash -c "bash rosbag_record.sh" & sleep 1
