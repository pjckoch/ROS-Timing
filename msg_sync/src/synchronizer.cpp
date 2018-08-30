/*
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.
LICENSE SUMMARY:
------------------------------------------
               BSD License
applies to:
- ros, Copyright (c) 2008, Willow Garage, Inc.
- sensor_msgs, Copyright (c) 2008, Willow Garage, Inc.
- audio_common_msgs, Copyright (c) 2008, Willow Garage, Inc.
- image_transport, Copyright (c) 2009, Willow Garage, Inc.
- message_filters, Copyright (c) 2009, Willow Garage, Inc.
------------------------------------------
*/

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <audio_proc/FFTData.h>
#include <audio_proc/AudioWav.h>

#include <image_transport/image_transport.h>

#include <iostream>


using namespace sensor_msgs;
using namespace message_filters;

ros::Duration dur;
ros::Time begin;


class SyncMessages {

public:
  SyncMessages() {

      ros::NodeHandle nh_; 
      image_transport::ImageTransport it_(nh_);
      
      // input topic names
      std::string audio_top;
      std::string fft_top;
      std::string img_top1;
      std::string img_top2;
      std::string img_top3;
      std::string img_top4;
      std::string img_top5;
      std::string img_top6;
      
      // name-extension for output topics
      std::string s;

      // queue size for the message filter
      int queue_size_;

      // get parameters from ROS parameter server
      
      nh_.param<int>("queue_size", queue_size_, 20);

      nh_.param<std::string>("audio_topic", audio_top, "/audio");
      nh_.param<std::string>("fft_topic", fft_top, "/fftData");
      nh_.param<std::string>("img_topic1", img_top1, "/stereo/right/image_raw");
      nh_.param<std::string>("img_topic2", img_top2, "/stereo/left/image_raw");
      nh_.param<std::string>("img_topic3", img_top3, "/stereo/right/edge_map");
      nh_.param<std::string>("img_topic4", img_top4, "/stereo/left/edge_map");
      nh_.param<std::string>("img_topic5", img_top5, "/stereo/right/blob");
      nh_.param<std::string>("img_topic6", img_top6, "/stereo/left/blob");

      // subscribe to topics
      audio_sub.subscribe(nh_, audio_top, 1);
      fft_sub.subscribe(nh_, fft_top, 1);
      img_sub1.subscribe(nh_, img_top1, 1); 
      img_sub2.subscribe(nh_, img_top2, 1); 
      img_sub3.subscribe(nh_, img_top3, 1); 
      img_sub4.subscribe(nh_, img_top4, 1); 
      img_sub5.subscribe(nh_, img_top5, 1); 
      img_sub6.subscribe(nh_, img_top6, 1);  

      s = "/sync";

      // advertise publisher topics
      audio_pub = nh_.advertise<audio_proc::AudioWav>(audio_top + s, 1);
      fft_pub = nh_.advertise<audio_proc::FFTData>(fft_top + s, 1);
      img_pub1 = it_.advertise(img_top1 + s, 1);
      img_pub2 = it_.advertise(img_top2 + s, 1);
      img_pub3 = it_.advertise(img_top3 + s, 1);
      img_pub4 = it_.advertise(img_top4 + s, 1);
      img_pub5 = it_.advertise(img_top5 + s, 1);
      img_pub6 = it_.advertise(img_top6 + s, 1);

      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
      sync_.reset(new Sync(MySyncPolicy(10), audio_sub, fft_sub, img_sub1, img_sub2,
                  img_sub3, img_sub4, img_sub5, img_sub6));
      // The number of arguments (topics) is limited to 8 because of limitations of boost::bind
      sync_->registerCallback(boost::bind(&SyncMessages::callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

  }

  void callback(const audio_proc::AudioWav::ConstPtr& audio, const audio_proc::FFTData::ConstPtr& fft,
                const ImageConstPtr& img1, const ImageConstPtr& img2,
                const ImageConstPtr& img3, const ImageConstPtr& img4,
                const ImageConstPtr& img5, const ImageConstPtr& img6) {   
        // Starts as soon as synchronized messages are available
        std::cout << "Image topics in sync" << std::endl;

        // republish everything in sync
        audio_pub.publish(audio);
        fft_pub.publish(fft);
        begin = ros::Time::now();
        img_pub1.publish(img1);
        img_pub2.publish(img2);
        img_pub3.publish(img3);
        img_pub4.publish(img4);
        img_pub5.publish(img5);
        img_pub6.publish(img6);
        dur = ros::Time::now() - begin;
        std::cout << dur.toSec();

    } 

private:
      // subscribers
      message_filters::Subscriber<audio_proc::AudioWav> audio_sub;
      message_filters::Subscriber<audio_proc::FFTData> fft_sub;
      message_filters::Subscriber<Image> img_sub1; 
      message_filters::Subscriber<Image> img_sub2; 
      message_filters::Subscriber<Image> img_sub3; 
      message_filters::Subscriber<Image> img_sub4; 
      message_filters::Subscriber<Image> img_sub5; 
      message_filters::Subscriber<Image> img_sub6;

      // publishers
      ros::Publisher audio_pub;
      ros::Publisher fft_pub;
      image_transport::Publisher img_pub1;
      image_transport::Publisher img_pub2;
      image_transport::Publisher img_pub3;
      image_transport::Publisher img_pub4;
      image_transport::Publisher img_pub5;
      image_transport::Publisher img_pub6;
      
      typedef sync_policies::ApproximateTime<audio_proc::AudioWav, audio_proc::FFTData, Image, Image, Image, Image, Image, Image> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
      typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronizer");
  SyncMessages sm;
  ros::spin();
  return 0;
}
