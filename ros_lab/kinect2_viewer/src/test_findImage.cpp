#include "kinect2_viewer/PointCloud.h"
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
#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>	//dxh
#include <pcl_conversions/pcl_conversions.h>	//dxh
#include <sensor_msgs/PointCloud2.h>	//dxh
#include <pcl/filters/approximate_voxel_grid.h>

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

  double cost, start, end;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kinect2_viewer::PointCloud>("/interact_proj/pointcloud_srvs", 100);
  kinect2_viewer::PointCloud srv;
  ros::Time tmp(atoll(argv[1]));
  srv.request.a = tmp;
  ROS_INFO("srv.request.a = %lf", tmp.toSec());
  start = clock();
  if (client.call(srv))
  {
      try
      {
/*
	srv.response.point_cloud.header.frame_id = "map";
	ros::NodeHandle nt;
	ros::Publisher pcd_pub = nt.advertise<sensor_msgs::PointCloud2>("pcd_test", 1000);
	ros::Rate loop_rate(10);
	while (ros::ok())
  	{
    	  pcd_pub.publish(srv.response.point_cloud);
    	  ros::spinOnce();
    	  loop_rate.sleep();
  	}

	ROS_INFO("point_cloud.header.frame_id is %d", srv.response.point_cloud.height);

        cv::Mat cv_frame = cv_bridge::toCvCopy( srv.response.color, "bgr8")->image;
        cv::imshow("view", cv_frame);
	cv::waitKey(0);

        cv::Mat cv_frame2 = cv_bridge::toCvCopy( srv.response.depth, "mono8")->image;
        cv::imshow("view2", cv_frame2);
	cv::waitKey(0);
*/
	end = clock();
	cost = end - start;
	ROS_INFO("cost = %lf", cost / CLOCKS_PER_SEC);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", srv.response.depth.encoding.c_str());
      }
  }
  else
  {
    ROS_ERROR("Failed to call service pointcloud_srvs");
    return 1;
  }

  return 0;
}
