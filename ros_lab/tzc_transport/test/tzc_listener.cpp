#include "ros/ros.h"
#include "tzc_topic.hpp"
#include "tzc_image.hpp"
#include <unistd.h>

using namespace tzc_transport;

void chatterCallback(const ImageConstPtr & msg) {
  ROS_INFO("I heard: [%d, %f] deltaT=%fms", *(uint32_t *)msg->data, msg->header.stamp.toSec(),
           (ros::Time::now() - msg->header.stamp).toSec() * 1000.0);
  // sleep(1); // pretend doing complex work
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "tzc_listener", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  Topic t(n);
  ImageSubscriber s = t.subscribe< Image >("tzc_test_topic", 1, &chatterCallback);
  ros::spin();
  return 0;
}

