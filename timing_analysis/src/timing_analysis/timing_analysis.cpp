/*
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.
LICENSE SUMMARY:
------------------------------------------
               BSD License
applies to:
- ros, Copyright (c) 2008, Willow Garage, Inc.
- std_msgs, Copyright (c) 2008, Willow Garage, Inc.
------------------------------------------
*/

#include <timing_analysis/timing_analysis.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>


void publishDuration(ros::Time stamp_sub, ros::Time callback_begin, ros::Time callback_end, ros::Publisher pub) {
    std_msgs::Float32MultiArray time_msg;
    ros::Duration elapsed_proc = callback_end - callback_begin;
    ros::Duration elapsed = callback_end - stamp_sub;
    time_msg.data.resize(2);
    time_msg.data[0] = elapsed_proc.toSec();
    time_msg.data[1] = elapsed.toSec();
    pub.publish(time_msg);
}
