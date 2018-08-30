# The msg_sync package

## Description

This catkin package serves to sychnronize multiple ROS topics. For instance, this can be useful if you want to synchronize an audio and a video stream.

This package contains one C++ program:
- [synchronizer.cpp](src/msg_sync/synchronizer.cpp): Subscribe to multiple (asynchronous) ROS topics and re-publish the received messages in sync. Synchronization is achieved by using a [message filter with ApproximateTime policy](http://wiki.ros.org/message_filters#ApproximateTime_Policy).

The current implementation synchronizes 7 topics with message type sensor_msgs::Image, 1 with message type audio_proc::AudioWav and 1 with audio_proc::FFTData. The latter two come with the audio_proc package of the [ROS-Robotic-Infant-Ears repository](https://github.com/pjckoch/ROS-Robotic-Infant-Ears.git).

You are free to adapt the code to your needs, i.e. you can change the number or type of messages to be synchronized and you can process the messages instead of merely re-publishing them.

## Prerequisites
- Clone this repository into your catkin workspace.
- Clone the [ROS-Robotic-Infant-Ears](https://github.com/pjckoch/ROS-Robotic-Infant-Ears.git) and the [ROS-Robotic-Infant-Eyes](https://github.com/pjckoch/ROS-Robotic-Infant-Eyes.git) repository into your catkin workspace, unless you want to modify [synchronizer.cpp](src/msg_sync/synchronizer.cpp) to synchronize other message types. 
- Build your catkin workspace.
- Remark: The message_filters package is part of the [ros_communication related packages](https://github.com/ros/ros_comm.git) and should already be installed in your ROS environment.

## How to use
1. `rosrun msg_sync synchronizer`

**Remark**: You can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

## ROS parameters

- **queue_size**: Queue size of the message filter.
- **img_topic1** to **img_topic7**: Any topic name under which messages of type sensor_msgs::Image are published.
- **audio_topic**: Topic name under which messages of type audio_proc::AudioWav are published.
- **fft_topic**: Topic name under which messages of type audio_proc::FFTData are published.

**Remark**: Remember that passing parameters with `rosrun` is slighty different from passing them with `roslaunch`. That is, instead of `roslaunch mylaunchfile myparam:=myvalue`, you would call `rosrun mynode _myparam:=myvalue`.

## License

This project is licensed under the 3-Clause-BSD-License (see the [LICENSE.md](../LICENSE/LICENSE.md) for details). For third-party licenses, see [LICENSE-3RD-PARTY.md](../LICENSE/LICENSE-3RD-PARTY.md).
