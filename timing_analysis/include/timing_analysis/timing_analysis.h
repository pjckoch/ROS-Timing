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

#ifndef __TIMING_ANALYSIS_H_
#define __TIMING_ANALYSIS_H_

#include <ros/ros.h>


void publishDuration(ros::Time stamp_sub, ros::Time process_begin, ros::Time callback_end, ros::Publisher pub);


#endif /* __TIMING_ANALYSIS_H_ */
