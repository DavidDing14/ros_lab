#ifndef __TZC_OBJECT_HPP__
#define __TZC_OBJECT_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/atomic/atomic.hpp>
#include <ros/ros.h>

namespace tzc_transport
{

// some typedef for short
typedef boost::atomic< int32_t > atomic_int32_t;
typedef boost::interprocess::managed_shared_memory ShmManager;
typedef boost::shared_ptr< boost::interprocess::managed_shared_memory > ShmManagerPtr;
typedef boost::interprocess::interprocess_mutex ipc_mutex;

class ShmMessage
{
public:
  ShmMessage(const ShmManagerPtr & pshm, uint32_t ref0) {
    long hc = pshm->get_handle_from_address(this);
    next = hc;
    prev = hc;
    ref = ref0;
    saveRef = 0;
  }
  ~ShmMessage() {
  }

  void addLast(ShmMessage * lc, const ShmManagerPtr & pshm) {
    long hc = pshm->get_handle_from_address(lc);
    long hn = pshm->get_handle_from_address(this), hp = this->prev;
    ShmMessage * ln = this, * lp = (ShmMessage *)pshm->get_address_from_handle(hp);
    lc->next = hn;
    lc->prev = hp;
    lp->next = hc;
    ln->prev = hc;
  }
  double releaseFirst(const ShmManagerPtr & pshm) {
    long hc = next;
    ShmMessage * lc = (ShmMessage *)pshm->get_address_from_handle(hc);
    if (lc == this)
      return -1;
    if (saveRef != 0)
    {
      ROS_INFO("saveRef != 0, assist node find this image");
      return -1;
    }
    double time_ = lc->getTimeStamp();
//    if (lc->ref != 0)
//      return false;
    long hn = lc->next, hp = lc->prev;
    ShmMessage * ln = (ShmMessage *)pshm->get_address_from_handle(hn), * lp = this;
    lp->next = hn;
    ln->prev = hp;
    pshm->deallocate(lc);
    return time_;
  }
  long getFirstHandle() {
    return next;
  }

  uint32_t take() {
    return ref.fetch_add(1, boost::memory_order_relaxed);
  }
  uint32_t release() {
    return ref.fetch_sub(1, boost::memory_order_relaxed);
  }
  void setRef(uint32_t ref0) {
    ref = ref0;
  }

  uint32_t getRef() {
    return ref;
  }

  uint32_t getNext() {
    return next;
  }

  uint32_t getPrev() {
    return prev;
  }

  void setTimeStamp(double setTime) {
    timeStamp = setTime;
  }

  double getTimeStamp() {
    return timeStamp;
  }

private:
  long next;
  long prev;
  atomic_int32_t ref;
  double timeStamp;	//image time
  atomic_int32_t saveRef;	//0 : can release / 1 : cannot release
};

class ShmObject
{
public:
  ShmObject(ShmManager * pshm, std::string name)
      : pshm_(pshm), name_(name) {
    plck_ = pshm_->find_or_construct< ipc_mutex >("lck")();
    pmsg_ = pshm_->find_or_construct< ShmMessage >("msg")(pshm_, 0);
    pmsg_->take();
  }

  ~ShmObject() {
    if (pmsg_->release() == 1) {
      boost::interprocess::shared_memory_object::remove(name_.c_str());
//      printf("shm file <%s> removed\n", name_.c_str());
    }
  }

  void addLast(ShmMessage * ptr) {
    plck_->lock();
    pmsg_->addLast(ptr, pshm_);
    ptr->setRef(1);
    plck_->unlock();
  }
  double releaseFirst() {
    plck_->lock();
    double res = pmsg_->releaseFirst(pshm_);
    ROS_INFO("res = %f", res);
    plck_->unlock();
    return res;
  }

  void * allocate(uint32_t size) {
    return pshm_->allocate(size);
  }
  void * convertHandle2Address(long h) {
    return pshm_->get_address_from_handle(h);
  }
  long convertAddress2Handle(void * p) {
    return pshm_->get_handle_from_address(p);
  }

  // in shm, message list and ref count (pub # + sub #)
  ShmMessage * pmsg_;

  // smart pointer of shm
  ShmManagerPtr pshm_;

private:
  // name of shm
  std::string name_;
  // in shm, connection lock
  ipc_mutex * plck_;
  
};
typedef boost::shared_ptr< ShmObject > ShmObjectPtr;

} // namespace tzc_transport

#endif // __TZC_OBJECT_HPP__

