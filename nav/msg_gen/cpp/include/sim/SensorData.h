/* Auto-generated by genmsg_cpp for file /home/cojabo/ros_workspace/sim/msg/SensorData.msg */
#ifndef SIM_MESSAGE_SENSORDATA_H
#define SIM_MESSAGE_SENSORDATA_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace sim
{
template <class ContainerAllocator>
struct SensorData_ {
  typedef SensorData_<ContainerAllocator> Type;

  SensorData_()
  : ultrasonic_frontLeft_distance(0)
  , ultrasonic_frontCenter_distance(0)
  , ultrasonic_frontRight_distance(0)
  , ultrasonic_rearRight_distance(0)
  , ultrasonic_rearCenter_distance(0)
  , ultrasonic_rearLeft_distance(0)
  , infrared_frontLeftLeft_distance(0.0)
  , infrared_frontLeftCenter_distance(0.0)
  , infrared_frontRightCenter_distance(0.0)
  , infrared_frontRightRight_distance(0.0)
  , infrared_right_distance(0.0)
  , infrared_rear_distance(0.0)
  , infrared_left_distance(0.0)
  , human_left_motion(0)
  , human_left_presence(0)
  , human_right_motion(0)
  , human_right_presence(0)
  {
  }

  SensorData_(const ContainerAllocator& _alloc)
  : ultrasonic_frontLeft_distance(0)
  , ultrasonic_frontCenter_distance(0)
  , ultrasonic_frontRight_distance(0)
  , ultrasonic_rearRight_distance(0)
  , ultrasonic_rearCenter_distance(0)
  , ultrasonic_rearLeft_distance(0)
  , infrared_frontLeftLeft_distance(0.0)
  , infrared_frontLeftCenter_distance(0.0)
  , infrared_frontRightCenter_distance(0.0)
  , infrared_frontRightRight_distance(0.0)
  , infrared_right_distance(0.0)
  , infrared_rear_distance(0.0)
  , infrared_left_distance(0.0)
  , human_left_motion(0)
  , human_left_presence(0)
  , human_right_motion(0)
  , human_right_presence(0)
  {
  }

  typedef uint8_t _ultrasonic_frontLeft_distance_type;
  uint8_t ultrasonic_frontLeft_distance;

  typedef uint8_t _ultrasonic_frontCenter_distance_type;
  uint8_t ultrasonic_frontCenter_distance;

  typedef uint8_t _ultrasonic_frontRight_distance_type;
  uint8_t ultrasonic_frontRight_distance;

  typedef uint8_t _ultrasonic_rearRight_distance_type;
  uint8_t ultrasonic_rearRight_distance;

  typedef uint8_t _ultrasonic_rearCenter_distance_type;
  uint8_t ultrasonic_rearCenter_distance;

  typedef uint8_t _ultrasonic_rearLeft_distance_type;
  uint8_t ultrasonic_rearLeft_distance;

  typedef float _infrared_frontLeftLeft_distance_type;
  float infrared_frontLeftLeft_distance;

  typedef float _infrared_frontLeftCenter_distance_type;
  float infrared_frontLeftCenter_distance;

  typedef float _infrared_frontRightCenter_distance_type;
  float infrared_frontRightCenter_distance;

  typedef float _infrared_frontRightRight_distance_type;
  float infrared_frontRightRight_distance;

  typedef float _infrared_right_distance_type;
  float infrared_right_distance;

  typedef float _infrared_rear_distance_type;
  float infrared_rear_distance;

  typedef float _infrared_left_distance_type;
  float infrared_left_distance;

  typedef uint16_t _human_left_motion_type;
  uint16_t human_left_motion;

  typedef uint16_t _human_left_presence_type;
  uint16_t human_left_presence;

  typedef uint16_t _human_right_motion_type;
  uint16_t human_right_motion;

  typedef uint16_t _human_right_presence_type;
  uint16_t human_right_presence;


private:
  static const char* __s_getDataType_() { return "sim/SensorData"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "0720f31a3880984759c88032ef6503d8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 ultrasonic_frontLeft_distance\n\
uint8 ultrasonic_frontCenter_distance\n\
uint8 ultrasonic_frontRight_distance\n\
uint8 ultrasonic_rearRight_distance\n\
uint8 ultrasonic_rearCenter_distance\n\
uint8 ultrasonic_rearLeft_distance\n\
\n\
float32 infrared_frontLeftLeft_distance\n\
float32 infrared_frontLeftCenter_distance\n\
float32 infrared_frontRightCenter_distance\n\
float32 infrared_frontRightRight_distance\n\
float32 infrared_right_distance\n\
float32 infrared_rear_distance\n\
float32 infrared_left_distance\n\
\n\
uint16 human_left_motion\n\
uint16 human_left_presence\n\
uint16 human_right_motion\n\
uint16 human_right_presence\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, ultrasonic_frontLeft_distance);
    ros::serialization::serialize(stream, ultrasonic_frontCenter_distance);
    ros::serialization::serialize(stream, ultrasonic_frontRight_distance);
    ros::serialization::serialize(stream, ultrasonic_rearRight_distance);
    ros::serialization::serialize(stream, ultrasonic_rearCenter_distance);
    ros::serialization::serialize(stream, ultrasonic_rearLeft_distance);
    ros::serialization::serialize(stream, infrared_frontLeftLeft_distance);
    ros::serialization::serialize(stream, infrared_frontLeftCenter_distance);
    ros::serialization::serialize(stream, infrared_frontRightCenter_distance);
    ros::serialization::serialize(stream, infrared_frontRightRight_distance);
    ros::serialization::serialize(stream, infrared_right_distance);
    ros::serialization::serialize(stream, infrared_rear_distance);
    ros::serialization::serialize(stream, infrared_left_distance);
    ros::serialization::serialize(stream, human_left_motion);
    ros::serialization::serialize(stream, human_left_presence);
    ros::serialization::serialize(stream, human_right_motion);
    ros::serialization::serialize(stream, human_right_presence);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, ultrasonic_frontLeft_distance);
    ros::serialization::deserialize(stream, ultrasonic_frontCenter_distance);
    ros::serialization::deserialize(stream, ultrasonic_frontRight_distance);
    ros::serialization::deserialize(stream, ultrasonic_rearRight_distance);
    ros::serialization::deserialize(stream, ultrasonic_rearCenter_distance);
    ros::serialization::deserialize(stream, ultrasonic_rearLeft_distance);
    ros::serialization::deserialize(stream, infrared_frontLeftLeft_distance);
    ros::serialization::deserialize(stream, infrared_frontLeftCenter_distance);
    ros::serialization::deserialize(stream, infrared_frontRightCenter_distance);
    ros::serialization::deserialize(stream, infrared_frontRightRight_distance);
    ros::serialization::deserialize(stream, infrared_right_distance);
    ros::serialization::deserialize(stream, infrared_rear_distance);
    ros::serialization::deserialize(stream, infrared_left_distance);
    ros::serialization::deserialize(stream, human_left_motion);
    ros::serialization::deserialize(stream, human_left_presence);
    ros::serialization::deserialize(stream, human_right_motion);
    ros::serialization::deserialize(stream, human_right_presence);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(ultrasonic_frontLeft_distance);
    size += ros::serialization::serializationLength(ultrasonic_frontCenter_distance);
    size += ros::serialization::serializationLength(ultrasonic_frontRight_distance);
    size += ros::serialization::serializationLength(ultrasonic_rearRight_distance);
    size += ros::serialization::serializationLength(ultrasonic_rearCenter_distance);
    size += ros::serialization::serializationLength(ultrasonic_rearLeft_distance);
    size += ros::serialization::serializationLength(infrared_frontLeftLeft_distance);
    size += ros::serialization::serializationLength(infrared_frontLeftCenter_distance);
    size += ros::serialization::serializationLength(infrared_frontRightCenter_distance);
    size += ros::serialization::serializationLength(infrared_frontRightRight_distance);
    size += ros::serialization::serializationLength(infrared_right_distance);
    size += ros::serialization::serializationLength(infrared_rear_distance);
    size += ros::serialization::serializationLength(infrared_left_distance);
    size += ros::serialization::serializationLength(human_left_motion);
    size += ros::serialization::serializationLength(human_left_presence);
    size += ros::serialization::serializationLength(human_right_motion);
    size += ros::serialization::serializationLength(human_right_presence);
    return size;
  }

  typedef boost::shared_ptr< ::sim::SensorData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sim::SensorData_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SensorData
typedef  ::sim::SensorData_<std::allocator<void> > SensorData;

typedef boost::shared_ptr< ::sim::SensorData> SensorDataPtr;
typedef boost::shared_ptr< ::sim::SensorData const> SensorDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::sim::SensorData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::sim::SensorData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace sim

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::sim::SensorData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::sim::SensorData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::sim::SensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0720f31a3880984759c88032ef6503d8";
  }

  static const char* value(const  ::sim::SensorData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0720f31a38809847ULL;
  static const uint64_t static_value2 = 0x59c88032ef6503d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::sim::SensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sim/SensorData";
  }

  static const char* value(const  ::sim::SensorData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::sim::SensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 ultrasonic_frontLeft_distance\n\
uint8 ultrasonic_frontCenter_distance\n\
uint8 ultrasonic_frontRight_distance\n\
uint8 ultrasonic_rearRight_distance\n\
uint8 ultrasonic_rearCenter_distance\n\
uint8 ultrasonic_rearLeft_distance\n\
\n\
float32 infrared_frontLeftLeft_distance\n\
float32 infrared_frontLeftCenter_distance\n\
float32 infrared_frontRightCenter_distance\n\
float32 infrared_frontRightRight_distance\n\
float32 infrared_right_distance\n\
float32 infrared_rear_distance\n\
float32 infrared_left_distance\n\
\n\
uint16 human_left_motion\n\
uint16 human_left_presence\n\
uint16 human_right_motion\n\
uint16 human_right_presence\n\
";
  }

  static const char* value(const  ::sim::SensorData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::sim::SensorData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::sim::SensorData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ultrasonic_frontLeft_distance);
    stream.next(m.ultrasonic_frontCenter_distance);
    stream.next(m.ultrasonic_frontRight_distance);
    stream.next(m.ultrasonic_rearRight_distance);
    stream.next(m.ultrasonic_rearCenter_distance);
    stream.next(m.ultrasonic_rearLeft_distance);
    stream.next(m.infrared_frontLeftLeft_distance);
    stream.next(m.infrared_frontLeftCenter_distance);
    stream.next(m.infrared_frontRightCenter_distance);
    stream.next(m.infrared_frontRightRight_distance);
    stream.next(m.infrared_right_distance);
    stream.next(m.infrared_rear_distance);
    stream.next(m.infrared_left_distance);
    stream.next(m.human_left_motion);
    stream.next(m.human_left_presence);
    stream.next(m.human_right_motion);
    stream.next(m.human_right_presence);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SensorData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sim::SensorData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::sim::SensorData_<ContainerAllocator> & v) 
  {
    s << indent << "ultrasonic_frontLeft_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_frontLeft_distance);
    s << indent << "ultrasonic_frontCenter_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_frontCenter_distance);
    s << indent << "ultrasonic_frontRight_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_frontRight_distance);
    s << indent << "ultrasonic_rearRight_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_rearRight_distance);
    s << indent << "ultrasonic_rearCenter_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_rearCenter_distance);
    s << indent << "ultrasonic_rearLeft_distance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ultrasonic_rearLeft_distance);
    s << indent << "infrared_frontLeftLeft_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_frontLeftLeft_distance);
    s << indent << "infrared_frontLeftCenter_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_frontLeftCenter_distance);
    s << indent << "infrared_frontRightCenter_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_frontRightCenter_distance);
    s << indent << "infrared_frontRightRight_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_frontRightRight_distance);
    s << indent << "infrared_right_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_right_distance);
    s << indent << "infrared_rear_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_rear_distance);
    s << indent << "infrared_left_distance: ";
    Printer<float>::stream(s, indent + "  ", v.infrared_left_distance);
    s << indent << "human_left_motion: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.human_left_motion);
    s << indent << "human_left_presence: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.human_left_presence);
    s << indent << "human_right_motion: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.human_right_motion);
    s << indent << "human_right_presence: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.human_right_presence);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SIM_MESSAGE_SENSORDATA_H

