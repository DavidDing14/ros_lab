#include "kinect2_viewer/UseStamp.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>	//dxh
#include <pcl_conversions/pcl_conversions.h>	//dxh
#include <sensor_msgs/PointCloud2.h>	//dxh

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_findImage");
  if (argc != 2)
  {
    ROS_INFO("usage: test_findImage stamp //stamp means the 'ros::Time'(time) of the image you want to find");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kinect2_viewer::UseStamp>("use_stamp", 100);
  kinect2_viewer::UseStamp srv;
  ros::Time tmp(atoll(argv[1]));
  srv.request.a = tmp;
  if (client.call(srv))
  {
      try
      {

        cv::Mat cv_frame = cv_bridge::toCvCopy( srv.response.color, "bgr8")->image;
        cv::imshow("view", cv_frame);
	cv::waitKey(0);

        cv::Mat cv_frame2 = cv_bridge::toCvCopy( srv.response.depth, "mono8")->image;
        cv::imshow("view2", cv_frame2);
	cv::waitKey(0);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", srv.response.depth.encoding.c_str());
      }
  }
  else
  {
    ROS_ERROR("Failed to call service use_stamp");
    return 1;
  }

  return 0;
}
