#ifndef __TZC_PUBLISHER_HPP__
#define __TZC_PUBLISHER_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "tzc_object.hpp"

namespace tzc_transport
{

class Topic;

template < class M >
class Publisher
{
public:
  Publisher() {
  }

  ~Publisher() {
  }

  Publisher(const Publisher & p) {
    *this = p;
  }

  Publisher & operator = (const Publisher & p) {
    pub_ = p.pub_;
    pobj_ = p.pobj_;
    return *this;
  }

  bool prepare(M & msg) const {
    if (!pobj_)
      return false;

    bool succeeded = false;
#define RETRY 2
    // allocation shm message
    ShmMessage * ptr = NULL;
    // bad_alloc exception may occur if some ros messages are lost
    int attempt = 0;
    for (; attempt < RETRY && ptr == NULL; attempt++) {
      try {
        ptr = msg.allocate(pobj_);
      } catch (boost::interprocess::bad_alloc e) {
        if (!pobj_->releaseFirst()) {
          ROS_WARN("the oldest is in use, abandon this message <%p>...", &msg);
          break;
        }
      }
    }
    if (ptr) {
      msg.fill(ptr, pobj_);
      succeeded = true;
    } else if (attempt >= RETRY) {
      ROS_WARN("bad_alloc happened %d times, abandon this message <%p>...", attempt, &msg);
    }
#undef RETRY
    return succeeded;
  }

  void publish(const M & msg) const {
    if (!pobj_)
      return;
    pub_.publish(msg);
  }

  void publish(const typename M::ConstPtr & msg) const {
    if (!pobj_)
      return;
    pub_.publish(msg);
  }

  void shutdown() {
    pub_.shutdown();
  }

  std::string getTopic() const {
    return pub_.getTopic();
  }

  uint32_t getNumSubscribers() const {
    return pub_.getNumSubscribers();
  }

private:
  Publisher(const ros::Publisher & pub, const std::string & topic, uint32_t mem_size)
      : pub_(pub) {
    // change '/' in topic to '_'
    std::string t = topic;
    for (int i = 0; i < t.length(); i++)
      if (t[i] == '/')
        t[i] = '_';
    ShmManager * pshm = new ShmManager(boost::interprocess::open_or_create, t.c_str(), mem_size);
    pobj_ = ShmObjectPtr(new ShmObject(pshm, t));
  }

  ros::Publisher pub_;
  ShmObjectPtr   pobj_;

friend class Topic;
};

} // namespace tzc_transport

#endif // __TZC_PUBLISHER_HPP__

