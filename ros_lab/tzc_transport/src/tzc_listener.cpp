#include "ros/ros.h"
#include "tzc_transport/tzc_topic.hpp"
#include "tzc_transport/tzc_image.hpp"
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <tzc_transport/findHandle.h>

using namespace tzc_transport;

class dataHandler	//dxh  save timeStamp and data_handle, so that we can find data_handle with timeStamp, and use data_handle with pobj to find data_address
{
public:
  ros::Time timeStamp;
  long data_handle;
public:
  dataHandler(ros::Time timeStamp, long data_handle) {
    this->timeStamp = timeStamp;
    this->data_handle = data_handle;
  }
  ~dataHandler() {
  }
};

std::vector<dataHandler> dataBuffer;	//dxh

void chatterCallback(const ImageConstPtr & msg) {
  //ROS_INFO("I heard: [%d, %f] deltaT=%fms \n data_handle=%ld  ref=%d", *(uint32_t *)msg->data, msg->header.stamp.toSec(),
  //         (ros::Time::now() - msg->header.stamp).toSec() * 1000.0, msg->data_handle, msg->pobj_->pmsg_->getRef());

  //ROS_INFO("msg->height = %d, msg->width = %d", msg->height, msg->width);

  dataHandler newData(msg->header.stamp, msg->data_handle);	//dxh
  if(dataBuffer.size()>=100)
  {
    dataBuffer.erase(dataBuffer.begin());
  }
  dataBuffer.push_back(newData);

//  long prev_handle = msg->pobj_->pmsg_->getPrev();
//  uint8_t * data = (uint8_t *)msg->pobj_->convertHandle2Address(prev_handle);
//show image that we get
//  cv::Mat rgbmat;

//  cv::Mat(msg->height, msg->width, CV_8UC4, data).copyTo(rgbmat);
//  cv::imshow("rgb", rgbmat);
//  cv::waitKey(0);
 

   //sleep(1); // pretend doing complex work
}

bool findHandle_(tzc_transport::findHandle::Request &req, tzc_transport::findHandle::Response &res)
{
  double reqtoSec = req.timeStamp.toSec();
  ROS_INFO("request: timeStamp = %f", reqtoSec);
  double minSec = fabs(dataBuffer[0].timeStamp.toSec() - reqtoSec);
  int imageNo = 0;
  for (int i=1; i<dataBuffer.size(); ++i)
  {
    ROS_INFO("%d %f %f ", i, dataBuffer[i].timeStamp.toSec(), fabs(dataBuffer[i].timeStamp.toSec() - reqtoSec));
    if(fabs(dataBuffer[i].timeStamp.toSec() - reqtoSec) < minSec)
    {
      minSec = fabs(dataBuffer[i].timeStamp.toSec() - reqtoSec);
      imageNo = i;
    }
  }

  res.data_handle = dataBuffer[imageNo].data_handle;

  ROS_INFO("sending back response: [%ld]", (long int)imageNo);
  ROS_INFO("dataBuffer[imageNo].timeStamp = %f", dataBuffer[imageNo].timeStamp.toSec());
  ROS_INFO("timediff = %lf", minSec);

  return true;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "tzc_listener", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  Topic t(n);
  //ImageSubscriber s = t.subscribe< Image >("tzc_test_topic", 1, &chatterCallback);
  ImageSubscriber s = t.subscribe< Image >("/kinect2/raw/color", 30, &chatterCallback);
  
  ros::NodeHandle nd;	//dxh
  ros::ServiceServer service = nd.advertiseService("find_Handle", findHandle_);	//dxh
  ROS_INFO("Ready to findImage with stamp");	//dxh

  ros::spin();
  return 0;
}

