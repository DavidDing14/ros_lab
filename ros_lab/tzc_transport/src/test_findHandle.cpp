#include <ros/ros.h>
#include <tzc_transport/findHandle.h>
#include <cstdlib>
#include <tzc_transport/tzc_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <opencv2/opencv.hpp>

#define depthheight 424
#define depthwidth 512
#define colorheight 1080
#define colorwidth 1920
#define mainTopic "_kinect2_raw_color"

using namespace tzc_transport;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_findHandle");
  if (argc != 2)
  {
    ROS_INFO("usage: test_findHandle timeStamp");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tzc_transport::findHandle>("find_Handle");
  tzc_transport::findHandle srv;
  ros::Time tmp(atoll(argv[1]));
  srv.request.timeStamp = tmp;
  //ROS_INFO("srv.request.timeStamp = %f", tmp.toSec());

  tzc_transport::ShmManager * pshm = new tzc_transport::ShmManager(boost::interprocess::open_only, mainTopic);
  tzc_transport::ShmObjectPtr pobj_ = tzc_transport::ShmObjectPtr(new tzc_transport::ShmObject(pshm, mainTopic));
  ros::Time start_ = ros::Time::now();
  if (client.call(srv))
  {
    ros::Time end_ = ros::Time::now();
    ros::Duration t_ = end_ - start_;
    ROS_INFO("time duration = %f", t_.toSec());
    long got_data_handle = srv.response.data_handle;
    //ROS_INFO("data_handle : %ld", got_data_handle);
/*	//dxh cannot find version
    if (got_data_handle == -1)
    {
      ROS_ERROR("cannot find the image");
      return 1;
    }
*/

    ShmManager * pshm = new tzc_transport::ShmManager(boost::interprocess::open_only, mainTopic);
    ShmMessage * msgData = (ShmMessage *)pshm->get_address_from_handle(got_data_handle);
    msgData->subSaveRef();

    uint8_t * data = (uint8_t *)pobj_->convertHandle2Address(got_data_handle);
    cv::Mat rgbmat;

    cv::Mat(colorheight, colorwidth, CV_8UC4, data).copyTo(rgbmat);
    cv::imshow("rgb", rgbmat);
    cv::waitKey(0);
  }
  else
  {
    ROS_ERROR("Failed to call service find_Handle");
    return 1;
  }

  return 0;
}
