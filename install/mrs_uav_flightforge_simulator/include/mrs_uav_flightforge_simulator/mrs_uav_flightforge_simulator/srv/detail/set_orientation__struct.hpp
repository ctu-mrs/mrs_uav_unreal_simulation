// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mrs_uav_flightforge_simulator/srv/set_orientation.hpp"


#ifndef MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_HPP_
#define MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Request __attribute__((deprecated))
#else
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Request __declspec(deprecated)
#endif

namespace mrs_uav_flightforge_simulator
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetOrientation_Request_
{
  using Type = SetOrientation_Request_<ContainerAllocator>;

  explicit SetOrientation_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : quaternion(_init)
  {
    (void)_init;
  }

  explicit SetOrientation_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : quaternion(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _quaternion_type =
    geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>;
  _quaternion_type quaternion;

  // setters for named parameter idiom
  Type & set__quaternion(
    const geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Request
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Request
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetOrientation_Request_ & other) const
  {
    if (this->quaternion != other.quaternion) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetOrientation_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetOrientation_Request_

// alias to use template instance with default allocator
using SetOrientation_Request =
  mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mrs_uav_flightforge_simulator


#ifndef _WIN32
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Response __attribute__((deprecated))
#else
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Response __declspec(deprecated)
#endif

namespace mrs_uav_flightforge_simulator
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetOrientation_Response_
{
  using Type = SetOrientation_Response_<ContainerAllocator>;

  explicit SetOrientation_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetOrientation_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Response
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Response
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetOrientation_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetOrientation_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetOrientation_Response_

// alias to use template instance with default allocator
using SetOrientation_Response =
  mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mrs_uav_flightforge_simulator


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Event __attribute__((deprecated))
#else
# define DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Event __declspec(deprecated)
#endif

namespace mrs_uav_flightforge_simulator
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetOrientation_Event_
{
  using Type = SetOrientation_Event_<ContainerAllocator>;

  explicit SetOrientation_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetOrientation_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mrs_uav_flightforge_simulator::srv::SetOrientation_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mrs_uav_flightforge_simulator::srv::SetOrientation_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Event
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mrs_uav_flightforge_simulator__srv__SetOrientation_Event
    std::shared_ptr<mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetOrientation_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetOrientation_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetOrientation_Event_

// alias to use template instance with default allocator
using SetOrientation_Event =
  mrs_uav_flightforge_simulator::srv::SetOrientation_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mrs_uav_flightforge_simulator

namespace mrs_uav_flightforge_simulator
{

namespace srv
{

struct SetOrientation
{
  using Request = mrs_uav_flightforge_simulator::srv::SetOrientation_Request;
  using Response = mrs_uav_flightforge_simulator::srv::SetOrientation_Response;
  using Event = mrs_uav_flightforge_simulator::srv::SetOrientation_Event;
};

}  // namespace srv

}  // namespace mrs_uav_flightforge_simulator

#endif  // MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_HPP_
