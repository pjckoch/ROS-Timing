#!/usr/bin/env python
"""
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.
LICENSE SUMMARY:
------------------------------------------
               BSD License
applies to:
- rospy, Copyright (c) 2008, Willow Garage, Inc.
- std_msgs, Copyright (c) 2008, Willow Garage, Inc.
------------------------------------------
"""

import rospy

from std_msgs.msg import Float32MultiArray

def publishDuration(stamp_sub, callback_begin, callback_end, pub):
    elapsed_proc = callback_end - callback_begin
    elapsed = callback_end - stamp_sub
    timearray = [elapsed_proc.to_sec(), elapsed.to_sec()]
    time_msg = Float32MultiArray(data=timearray)

    pub.publish(time_msg)
