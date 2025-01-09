#!/bin/bash

# Source ROS setup script
source /opt/ros/noetic/setup.zsh

# Specify the topics you want to record
TOPICS="/IMU_data /nlink_linktrack_nodeframe3 /mavros/px4flow/raw/optical_flow_rad /vicon_UAV/CSJ01/odom /vicon_UAV/CSJ01/pose /vicon_UAV/CSJ02/odom /vicon_UAV/CSJ02/pose /mavros/imu/data /mavros/local_position/pose  /mavros/setpoint_raw/attitude /mavros/vision_pose/pose /pva_setpoint /rosbag/MHEdata/dt /rosbag/MHEdata/flow /rosbag/MHEdata/uwb /rosbag/MHEdata/vicon01 /rosbag/MHEdata/vicon02 /rosbag/Realdata/gtd /rosbag/Realdata/vel /rosbag/Realdata/xtest /rosbag/MHEdata/height /rosbag/xt/pose /rosbag/xt/vel /rosbag/mavros/vision_pose/pose /estimated/MHEdata/dt /estimated/MHEdata/flow /estimated/MHEdata/uwb /estimated/MHEdata/vicon01 /estimated/MHEdata/vicon02 /estimated/Realdata/gtd /estimated/Realdata/vel /estimated/Realdata/xtest /estimated/MHEdata/height /estimated/xt/pose /estimated/xt/vel /estimated/mavros/vision_pose/pose"

# Set the output bag file name to the current date and time (精确到分钟)
OUTPUT_BAG_FILE=$(date +"../data/rosbag_record/ros.bag")

# Your rosbag record command here
rosbag record $TOPICS -o $OUTPUT_BAG_FILE
