// Generated by gencpp from file uol_cmp9767m_tutorial/AddTwoIntsResponse.msg
// DO NOT EDIT!


#ifndef UOL_CMP9767M_TUTORIAL_MESSAGE_ADDTWOINTSRESPONSE_H
#define UOL_CMP9767M_TUTORIAL_MESSAGE_ADDTWOINTSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace uol_cmp9767m_tutorial
{
template <class ContainerAllocator>
struct AddTwoIntsResponse_
{
  typedef AddTwoIntsResponse_<ContainerAllocator> Type;

  AddTwoIntsResponse_()
    : sum(0)  {
    }
  AddTwoIntsResponse_(const ContainerAllocator& _alloc)
    : sum(0)  {
  (void)_alloc;
    }



   typedef int64_t _sum_type;
  _sum_type sum;





  typedef boost::shared_ptr< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct AddTwoIntsResponse_

typedef ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<std::allocator<void> > AddTwoIntsResponse;

typedef boost::shared_ptr< ::uol_cmp9767m_tutorial::AddTwoIntsResponse > AddTwoIntsResponsePtr;
typedef boost::shared_ptr< ::uol_cmp9767m_tutorial::AddTwoIntsResponse const> AddTwoIntsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator1> & lhs, const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.sum == rhs.sum;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator1> & lhs, const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace uol_cmp9767m_tutorial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b88405221c77b1878a3cbbfff53428d7";
  }

  static const char* value(const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb88405221c77b187ULL;
  static const uint64_t static_value2 = 0x8a3cbbfff53428d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uol_cmp9767m_tutorial/AddTwoIntsResponse";
  }

  static const char* value(const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 sum\n"
"\n"
;
  }

  static const char* value(const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddTwoIntsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::uol_cmp9767m_tutorial::AddTwoIntsResponse_<ContainerAllocator>& v)
  {
    s << indent << "sum: ";
    Printer<int64_t>::stream(s, indent + "  ", v.sum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UOL_CMP9767M_TUTORIAL_MESSAGE_ADDTWOINTSRESPONSE_H
