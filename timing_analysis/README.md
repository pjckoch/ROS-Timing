# The timing_analysis package

## Description
This catkin package serves to analyze the timing of a ROS publisher-subscriber-pair. It requires that all messages of interest contain a header (std_msgs/Header). It will then publish the time that the subcriber node took for bare processing, and the elapsed time between setting the header timestamp (by the publisher) and finishing message processing in a callback (by the subscriber). This is particularly useful in distributed systems, where you would like to know how fast your node is and how much impact the connection between the system components (e.g. Wifi) has on the overall run-time.

## Prerequisites
- Install [chrony](https://chrony.tuxfamily.org/index.html) on each machine that is used: `apt-get install chrony`. After installation, the chrony daemon will start automatically and synchronize the system clock of the machine with various internet servers. In a distributed system, this ensures that the clocks of the machines are in sync. This is important, as otherwise a timing analysis would be meaningless.
- Compile [logfile_stats.cpp](src/timing_analysis/logfile_stats.cpp): `g++ -std=c++11 /path/to/logfile_stats.cpp -o logfile_stats`

## How to use

**Remark**: If you only intend to use this package with the [ROS-Robotic-Infant-Ears](https://github.com/pjckoch/ROS-Robotic-Infant-Ears.git) or the [ROS-Robotic-Infant-Eyes](https://github.com/pjckoch/ROS-Robotic-Infant-Eyes.git) repository, you can skip to step 8.

There is no need for to change anything about the publisher (as long as it publishes timestamped messages). All of the following steps refer to the subscriber only.

1. Add a build dependency on the timing_analysis package to the package.xml of your subscriber `<build_depend>timing_analysis</build_depend>`.
2. Find the timing_analysis package in your CMakeLists.txt: find_package(catkin REQUIRED COMPONENTS timing_analysis)

(Depending on which language your subscriber is implemented in, follow steps 3-7 either for C++ or Python.)

### C++

3. Call `#include <timing_analysis/timing_analysis.h>` at the top of the subcriber source code.
4. Create a `ros::Publisher` (called e.g. `time_pub_`) that will publish the duration. Advertise it under a topic name like "mySubscriberDuration".
5. At the beginning of the subscriber callback function, obtain the current system time, e.g. `ros::Time callback_begin = ros::Time::now();`
6. At the end of the callback, obtain the current system time `ros::Time callback_end = ros::Time::now();`
7. Pass the timestamp of the message that the node received `msg->header.stamp`, `callback_begin`, `callback_end` and `time_pub_` to the function `publishDuration()` which is contained in timing_analysis.h.


### Python
3. Call `from timing_analysis.python_timing_analysis import publishDuration` at the top of the subscriber source code.
4. Create a `rospy.Publisher` (called e.g. `time_pub_`) that will publish the duration. Advertise it under a topic name like "mySubscriberDuration".
5. At the beginning of the subscriber callback function, obtain the current system time, e.g. `callback_begin = rospy.Time.now()`.
6. At the end of the callback, obtain the current system time `callback_end = rospy.Time.now()`
7. Pass the timestamp of the message that the node received `msg.header.stamp`, `callback_begin`, `callback_end` and `time_pub_` to the function `publishDuration()` which you have imported in step 3.

(The following steps are language independent.)

8. The function `publishDuration()` will publish a `std_msgs::Float32MultiArray` which will contain the information:
[processing_time , processing_time+network_latency].
9. As soon as your node is running, you can the start the measurement by echoing the published duration messages into a txt- or log-file by running `rostopic echo -p /mySubscriberDuration > data.txt` on your command line.
10. When you finished your measurement, run the logfile_stats script (`./logfile_stats` on command line). You will be asked to specify the full path to your log-file (e.g. `/home/myusername/mylogfiles/mysubscriberduration.log`). Then, the results will be printed to your terminal.
