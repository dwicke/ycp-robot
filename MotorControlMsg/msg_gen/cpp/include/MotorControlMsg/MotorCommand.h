/* Auto-generated by genmsg_cpp for file /home/drew/git/ycp-robot/MotorControlMsg/msg/MotorCommand.msg */
#ifndef MOTORCONTROLMSG_MESSAGE_MOTORCOMMAND_H
#define MOTORCONTROLMSG_MESSAGE_MOTORCOMMAND_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"

namespace MotorControlMsg
{
template <class ContainerAllocator>
struct MotorCommand_ : public ros::Message
{
  typedef MotorCommand_<ContainerAllocator> Type;

  MotorCommand_()
  : header()
  , precedence(0)
  , isLeftRightVel(false)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  MotorCommand_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , precedence(0)
  , isLeftRightVel(false)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _precedence_type;
  int32_t precedence;

  typedef uint8_t _isLeftRightVel_type;
  uint8_t isLeftRightVel;

  typedef float _linear_velocity_type;
  float linear_velocity;

  typedef float _angular_velocity_type;
  float angular_velocity;


private:
  static const char* __s_getDataType_() { return "MotorControlMsg/MotorCommand"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "435226135b8d7ba320f037f1609e6fc6"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
\n\
int32 precedence\n\
bool isLeftRightVel\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, precedence);
    ros::serialization::serialize(stream, isLeftRightVel);
    ros::serialization::serialize(stream, linear_velocity);
    ros::serialization::serialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, precedence);
    ros::serialization::deserialize(stream, isLeftRightVel);
    ros::serialization::deserialize(stream, linear_velocity);
    ros::serialization::deserialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(precedence);
    size += ros::serialization::serializationLength(isLeftRightVel);
    size += ros::serialization::serializationLength(linear_velocity);
    size += ros::serialization::serializationLength(angular_velocity);
    return size;
  }

  typedef boost::shared_ptr< ::MotorControlMsg::MotorCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::MotorControlMsg::MotorCommand_<ContainerAllocator>  const> ConstPtr;
}; // struct MotorCommand
typedef  ::MotorControlMsg::MotorCommand_<std::allocator<void> > MotorCommand;

typedef boost::shared_ptr< ::MotorControlMsg::MotorCommand> MotorCommandPtr;
typedef boost::shared_ptr< ::MotorControlMsg::MotorCommand const> MotorCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::MotorControlMsg::MotorCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::MotorControlMsg::MotorCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace MotorControlMsg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::MotorControlMsg::MotorCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "435226135b8d7ba320f037f1609e6fc6";
  }

  static const char* value(const  ::MotorControlMsg::MotorCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x435226135b8d7ba3ULL;
  static const uint64_t static_value2 = 0x20f037f1609e6fc6ULL;
};

template<class ContainerAllocator>
struct DataType< ::MotorControlMsg::MotorCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "MotorControlMsg/MotorCommand";
  }

  static const char* value(const  ::MotorControlMsg::MotorCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::MotorControlMsg::MotorCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
int32 precedence\n\
bool isLeftRightVel\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::MotorControlMsg::MotorCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::MotorControlMsg::MotorCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::MotorControlMsg::MotorCommand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::MotorControlMsg::MotorCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.precedence);
    stream.next(m.isLeftRightVel);
    stream.next(m.linear_velocity);
    stream.next(m.angular_velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MotorCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::MotorControlMsg::MotorCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::MotorControlMsg::MotorCommand_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "precedence: ";
    Printer<int32_t>::stream(s, indent + "  ", v.precedence);
    s << indent << "isLeftRightVel: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isLeftRightVel);
    s << indent << "linear_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.linear_velocity);
    s << indent << "angular_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MOTORCONTROLMSG_MESSAGE_MOTORCOMMAND_H

