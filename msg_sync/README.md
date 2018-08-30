# The msg_sync package

## Description

This catkin package serves to subscribe to multiple (asynchronous) ROS topics and re-publish them in sync. For instance, this can be useful if you want to synchronize an audio and a video stream.

This package contains one C++ program:
- [synchronizer.cpp](src/msg_sync/synchronizer.cpp)

The current implementation synchronizes 7 topics with message type sensor_msgs::Image, 1 with message type audio_proc::AudioWav and 1 with audio_proc::FFTData. The latter two come with the audio_proc package of the [ROS-Robotic-Infant-Ears repository](https://github.com/pjckoch/ROS-Robotic-Infant-Ears.git).

You are free to adapt the code to your needs, i.e. you can change the number or type of messages to be synchronized and you can process the messages instead of merely re-publishing them.
