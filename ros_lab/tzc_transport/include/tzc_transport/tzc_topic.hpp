#ifndef __TZC_TOPIC_HPP__
#define __TZC_TOPIC_HPP__

#include "ros/ros.h"
#include "tzc_publisher.hpp"
#include "tzc_subscriber.hpp"
#include "std_msgs/Float64.h"

namespace tzc_transport
{

class Topic
{
public:
  Topic(const ros::NodeHandle & parent, const ros::NodeHandle & parent_p) {
    nh_ = parent;
    np_ = parent_p;
  }

  Topic(const ros::NodeHandle & parent) {
    nh_ = parent;
  }

  ~Topic() {
  }

  template < class M >
  Publisher< M > advertise(const std::string & topic, uint32_t queue_size, uint32_t mem_size) {
    ros::Publisher pub = nh_.advertise< M >(topic, queue_size);
    ros::Publisher pub_time = np_.advertise< std_msgs::Float64 >("release_time", 30);
    return Publisher< M >(pub, topic, mem_size, pub_time);
  }

  template < class M >
  Subscriber< M > subscribe(const std::string & topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr< const M > &)) {
    SubscriberCallbackHelper< M > * phlp = new SubscriberCallbackHelper< M >(topic, fp);
    ros::Subscriber sub = nh_.subscribe(topic, queue_size, &SubscriberCallbackHelper< M >::callback, phlp, ros::TransportHints().tcpNoDelay());
    return Subscriber< M >(sub, phlp);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle np_;
};

} // namespace tzc_transport

#endif // __TZC_TOPIC_HPP__


