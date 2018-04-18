#include "ros/ros.h"
#include "tzc_transport/tzc_topic.hpp"
#include "tzc_transport/tzc_image.hpp"

#define HEIGHT (1920)
#define WIDTH (1080)
#define SZ (HEIGHT * WIDTH * 3)
#define HZ (30)

using namespace tzc_transport;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "tzc_talker", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  Topic t(n);
  //ImagePublisher p = t.advertise< Image >("tzc_test_topic", 1, HZ * SZ);
  ImagePublisher p = t.advertise< Image >("/kinect2/raw/color", 1, HZ * SZ);

  ros::Rate loop_rate(HZ);
  int count = 0;

  Image img;
  img.height = HEIGHT;
  img.width = WIDTH;
  img.encoding = "rgb8";
  img.is_bigendian = 0;
  img.step = WIDTH * 3;
  img.data_size = SZ;


  while (ros::ok()) {
    loop_rate.sleep();

    img.header.stamp = ros::Time::now();
    if (p.prepare(img)) {
      int * pd = (int *)img.data;
      for (int i = 0; i < SZ / 4; i++)
        *(pd++) = count;

      p.publish(img);
      ROS_INFO("info: message [%d, %f] published", count, img.header.stamp.toSec());
    } else {
      ROS_WARN("warn: message [%d, %f] publish failed", count, img.header.stamp.toSec());
    }

    ros::spinOnce();
    count++;
  }
  return 0;
}

