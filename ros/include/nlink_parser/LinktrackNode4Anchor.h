// Generated by gencpp from file nlink_parser/LinktrackNode4Anchor.msg
// DO NOT EDIT!


#ifndef NLINK_PARSER_MESSAGE_LINKTRACKNODE4ANCHOR_H
#define NLINK_PARSER_MESSAGE_LINKTRACKNODE4ANCHOR_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nlink_parser
{
template <class ContainerAllocator>
struct LinktrackNode4Anchor_
{
  typedef LinktrackNode4Anchor_<ContainerAllocator> Type;

  LinktrackNode4Anchor_()
    : id(0)
    , dis(0.0)  {
    }
  LinktrackNode4Anchor_(const ContainerAllocator& _alloc)
    : id(0)
    , dis(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef float _dis_type;
  _dis_type dis;





  typedef boost::shared_ptr< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> const> ConstPtr;

}; // struct LinktrackNode4Anchor_

typedef ::nlink_parser::LinktrackNode4Anchor_<std::allocator<void> > LinktrackNode4Anchor;

typedef boost::shared_ptr< ::nlink_parser::LinktrackNode4Anchor > LinktrackNode4AnchorPtr;
typedef boost::shared_ptr< ::nlink_parser::LinktrackNode4Anchor const> LinktrackNode4AnchorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator1> & lhs, const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.dis == rhs.dis;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator1> & lhs, const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace nlink_parser

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3449514afa3f54cb0e542c3e3a394e77";
  }

  static const char* value(const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3449514afa3f54cbULL;
  static const uint64_t static_value2 = 0x0e542c3e3a394e77ULL;
};

template<class ContainerAllocator>
struct DataType< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nlink_parser/LinktrackNode4Anchor";
  }

  static const char* value(const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n"
"float32 dis\n"
;
  }

  static const char* value(const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.dis);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinktrackNode4Anchor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nlink_parser::LinktrackNode4Anchor_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "dis: ";
    Printer<float>::stream(s, indent + "  ", v.dis);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NLINK_PARSER_MESSAGE_LINKTRACKNODE4ANCHOR_H
