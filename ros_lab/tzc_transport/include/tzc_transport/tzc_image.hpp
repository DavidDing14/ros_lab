#ifndef __TZC_IMAGE_HPP__
#define __TZC_IMAGE_HPP__

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "tzc_topic.hpp"
#include "tzc_object.hpp"

namespace tzc_transport
{

template <class ContainerAllocator>
class Image_
{
public:
  Image_() : data(NULL), pobj_() { }

  Image_(const Image_ & img) {
    *this = img;
  }

  ~Image_() {
    if (pobj_ && data) {
      getMeta()->release();
    }
  }

  Image_ & operator = (const Image_ & img) {
    ShmObjectPtr old_pobj = pobj_;
    uint8_t *old_data = data;
  
    header = img.header;
    height = img.height;
    width = img.width;
    encoding = img.encoding;
    is_bigendian = img.is_bigendian;
    step = img.step;
    data = img.data;
    data_size = img.data_size;
    data_handle = img.data_handle;

    pobj_ = img.pobj_;
    // take new message
    if (pobj_ && data) {
      getMeta()->take();
    }
    // and release old message (order matters)
    if (old_pobj && old_data) {
      getMeta()->release();
    }
    return *this;
  }

private:
  // used by subscriber
  void setOwner(ShmObjectPtr pobj) {
    pobj_ = pobj;
    data = (uint8_t *)pobj->convertHandle2Address(data_handle);
    getMeta()->take();
  }
  // used by publisher
  ShmMessage * allocate(ShmObjectPtr pobj) {
    return (ShmMessage *)pobj->allocate(sizeof(ShmMessage) + data_size);
  }
  void fill(ShmMessage * ptr, ShmObjectPtr pobj) {
    if (pobj_ && data) {
      getMeta()->release();
    }
    pobj_ = pobj;
    data = (uint8_t *)(ptr + 1);
    data_handle = pobj_->convertAddress2Handle(data);
    ptr->setTimeStamp(this->header.stamp.toSec());
    pobj_->addLast(ptr);
  }
  // find parent ShmMessage
  ShmMessage * getMeta() {
    return (ShmMessage *)(data - sizeof(ShmMessage));
  }

public:
  std_msgs::Header header;
  uint32_t         height;
  uint32_t         width;
  std::string      encoding;
  uint8_t          is_bigendian;
  uint32_t         step;
  uint8_t *        data;
  uint32_t         data_size;
  long             data_handle;

  ShmObjectPtr pobj_;

private:
  

  friend class Publisher< Image_<ContainerAllocator> >;
  friend class SubscriberCallbackHelper< Image_<ContainerAllocator> >;

  typedef boost::shared_ptr< ::tzc_transport::Image_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tzc_transport::Image_<ContainerAllocator> const> ConstPtr;

}; // class Image_

typedef ::tzc_transport::Image_< std::allocator<void> > Image;

typedef boost::shared_ptr< ::tzc_transport::Image > ImagePtr;
typedef boost::shared_ptr< ::tzc_transport::Image const> ImageConstPtr;

typedef ::tzc_transport::Publisher< Image > ImagePublisher;
typedef ::tzc_transport::Subscriber< Image > ImageSubscriber;

} // tzc_transport

namespace ros
{

namespace message_traits
{

template<class ContainerAllocator>
struct IsFixedSize< tzc_transport::Image_<ContainerAllocator> >
  : FalseType
  { };

template<class ContainerAllocator>
struct IsFixedSize< tzc_transport::Image_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct IsMessage< tzc_transport::Image_<ContainerAllocator> >
  : TrueType
  { };

template<class ContainerAllocator>
struct IsMessage< tzc_transport::Image_<ContainerAllocator> const>
  : TrueType
  { };

template<class ContainerAllocator>
struct HasHeader< tzc_transport::Image_<ContainerAllocator> >
  : FalseType
  { };

template<class ContainerAllocator>
struct HasHeader< tzc_transport::Image_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< tzc_transport::Image_<ContainerAllocator> >
{
  static const char* value()
  {
    return "66613edaa0b43e60fce998aaeab97161";
  }

  static const char* value(const ::tzc_transport::Image_<ContainerAllocator> &) { return value(); }
  static const uint64_t static_value1 = 0x66613edaa0b43e60ULL;
  static const uint64_t static_value2 = 0xfce998aaeab97161ULL;
};

template<class ContainerAllocator>
struct DataType< tzc_transport::Image_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tzc_transport/Image";
  }

  static const char* value(const ::tzc_transport::Image_<ContainerAllocator> &) { return value(); }
};

template<class ContainerAllocator>
struct Definition< tzc_transport::Image_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\
  Header  header\n\
  uint32  height\n\
  uint32  width\n\
  string  encoding\n\
  uint8   is_bigendian\n\
  uint32  step\n\
  uint8[] data\n";
  }

  static const char* value(const ::tzc_transport::Image_<ContainerAllocator> &) { return value(); }
};

} // namespace message_traits

} // namespace ros

namespace ros
{

namespace serialization
{

  template<class ContainerAllocator>
  struct Serializer< tzc_transport::Image_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.height);
      stream.next(m.width);
      stream.next(m.encoding);
      stream.next(m.is_bigendian);
      stream.next(m.step);
      stream.next(m.data_size);
      stream.next(m.data_handle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  };

} // namespace serialization

} // namespace ros

#endif // __TZC_IMAGE_HPP__

